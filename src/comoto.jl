import Pkg; Pkg.activate("..");

using RobotDynamics, Rotations
using TrajectoryOptimization
using StaticArrays, LinearAlgebra
using RigidBodyDynamics;
using Altro;
import ForwardDiff;
using Dates;

include("ros_interface.jl")
include("problem_info.jl")

const RBD = RigidBodyDynamics;
const TO = TrajectoryOptimization;

struct Kuka <: TrajectoryOptimization.AbstractModel
    id::Int32
end

RobotDynamics.control_dim(::Kuka) = 7
RobotDynamics.state_dim(::Kuka) = 7

function RobotDynamics.dynamics(model::Kuka, x, u)
    u
end

"""
End effector constraint struct
n: state dimensionality
n_joints: number of robot joints 
lower_bounds: lower bounds for (x,y,z) eef position
upper_bounds: upper bounds for (x,y,z) eef position
fk: fk function (joint state)->cartesian position
"""
struct PosEECons <: TO.StateConstraint
    n::Int
    n_joints::Int
    lower_bounds::SArray{Tuple{3}, Float64}
    upper_bounds::SArray{Tuple{3}, Float64}
    fk::Function
end

TO.state_dim(con::PosEECons) = con.n
TO.sense(::PosEECons) = TO.Inequality()
Base.length(con::PosEECons) = 6*con.n_joints
function TO.evaluate(cons::PosEECons, x::StaticVector{l, T}) where {l, T}
    xyz = cons.fk(x)
    ans = T[]

    curr_idx = 1;
    for i = 1:cons.n_joints
        joint_pos = xyz[:,i]
        append!(ans, cons.lower_bounds - joint_pos)
        append!(ans, joint_pos - cons.upper_bounds)
        curr_idx += 6;
    end

    return SArray{Tuple{6*cons.n_joints}, T}(ans)
end

abstract type GeneralCostFunction{n,m} <: TO.CostFunction end
is_blockdiag(::GeneralCostFunction) = false
state_dim(::GeneralCostFunction{n}) where n = n
control_dim(::GeneralCostFunction{<:Any,m}) where m = m

mutable struct GeneralCost{n,m} <: GeneralCostFunction{n,m}
    cost_fn::Function
    is_terminal::Bool
end

function TO.stage_cost(cost::GeneralCost, x::AbstractVector)
    @assert cost.is_terminal
    cost.cost_fn(x)
end

function TO.stage_cost(cost::GeneralCost, x::AbstractVector, u::AbstractVector)
    @assert !cost.is_terminal
    cost.cost_fn(x, u)
end


"""
    gradient!(E::QuadraticCostFunction, costfun::CostFunction, x, u)
    gradient!(E::QuadraticCostFunction, costfun::CostFunction, x)
Evaluate the gradient of the cost function `costfun` at state `x` and control `u`, storing
    the result in `E.q` and `E.r`. Return a `true` if the gradient is constant, and `false`
    otherwise.
"""
function TO.gradient!(E::TO.QuadraticCostFunction, cost::GeneralCostFunction, x::AbstractVector)
    @assert cost.is_terminal
    grad = ForwardDiff.gradient(cost.cost_fn, x)
    E.q .= grad
    return false
end

function TO.gradient!(E::TO.QuadraticCostFunction, cost::GeneralCostFunction, x::AbstractVector, u::AbstractVector)
    @assert !cost.is_terminal
    state_lambda = t -> cost.cost_fn(t, u)
    ctrl_lambda = t -> cost.cost_fn(x, t)
    E.q .= ForwardDiff.gradient(state_lambda, x)
    E.r .= ForwardDiff.gradient(ctrl_lambda, u)
    if isnan(sum(E.q) + sum(E.r))
        println("It's a nan!")
    end
    return false
end

"""
    hessian!(E::QuadraticCostFunction, costfun::CostFunction, x, u)
    hessian!(E::QuadraticCostFunction, costfun::CostFunction, x)
Evaluate the hessian of the cost function `costfun` at state `x` and control `u`, storing
    the result in `E.Q`, `E.R`, and `E.H`. Return a `true` if the hessian is constant, and `false`
    otherwise.
"""
function TO.hessian!(E::TO.QuadraticCostFunction, cost::GeneralCostFunction, x::AbstractVector)
    @assert cost.is_terminal
    E.Q .= ForwardDiff.hessian(cost.cost_fn, x)
    return false
end

function TO.hessian!(E::TO.QuadraticCostFunction, cost::GeneralCostFunction, x::AbstractVector, u::AbstractVector)
    @assert !cost.is_terminal
    state_lambda = t -> cost.cost_fn(t, u)
    ctrl_lambda = t -> cost.cost_fn(x, t)
    E.Q .= ForwardDiff.hessian(state_lambda, x)
    E.R .= ForwardDiff.hessian(ctrl_lambda, u)
    return false
end

import Base
Base.copy(c::GeneralCost) = c

"""
CompoundCost implements a weighted sum of other costs
"""
function CompoundCost(cost_list::AbstractVector{<:TO.CostFunction}, is_terminal::Bool, n, m, weights=nothing)
    if isnothing(weights)
        weights = ones(length(cost_list))
    end
    @assert length(weights) == length(cost_list);
    if is_terminal
        return GeneralCost{n,m}(x -> dot([TO.stage_cost(c, x) for c in cost_list], weights), is_terminal)
    else
        return GeneralCost{n,m}((x, u) -> dot([TO.stage_cost(c, x, u) for c in cost_list], weights), is_terminal)
    end
end

function sq_norm(x::AbstractVector)
    dot(x, x)
end

"""
Returns the legibility cost for a given timestep

jacobian: function that accepts a joint state and returns the Jacobian matrix J
dt: the time allotted to one timestep (eg, 1.0 sec/timestep)
curr_timestep: the current timestep
total_timesteps: the total number of timesteps in the trajectory
goalset: 3xN matrix possible goals - first element (goalset[:,1]) is "true goal"
start: initial cartesian position
ee_fn: end effector fk function
"""
function legibility_costfn(x::AbstractVector, u::AbstractVector, dt::Float64, curr_timestep::Int, total_timesteps::Int, goalset::AbstractMatrix, start::AbstractVector, ee_fn::Function)
    # exp(-C(s->q) - C*(q->g))/exp(-C*(s->g))
    # = exp(-C(s->q) - C*(q->g) + C(s->g))
    cart_x = ee_fn(x);
    n_goals = size(goalset)[2];
    # create matrices that we can subtract from goalset
    start_rep, x_rep = repeat(start, outer=[1,n_goals]), repeat(cart_x, outer=[1,n_goals])
    start_goal_velset = (goalset - start_rep)/(dt * total_timesteps);
    nom_goal_velset = (goalset - x_rep)/(dt * (total_timesteps - curr_timestep));

    goal_weight = exp(-sq_norm(nom_goal_velset[:,1]) + sq_norm(start_goal_velset[:,1]));
    total_weight = goal_weight;
    for i = 2:n_goals
        total_weight += exp(-sq_norm(nom_goal_velset[:,i]) + sq_norm(start_goal_velset[:,i]));
    end
    leg = goal_weight/total_weight    
    if isnan(leg)
        println("NaN in legibility cost");
        if isa(curr_velnorm, Float64)
            println(start_goal_velset, nom_goal_velset);
        else
            println(start_goal_velset, nom_goal_velset);
        end
    end
    1-leg
end

"""
Returns an array of length total_timesteps of legibility cost functions
"""
function get_legibility_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = GeneralCost{n,m}[];
    cart_start = info.eef_fk(info.joint_start);
    for i = 0:(info.n_timesteps - 2)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> legibility_costfn(x, u, info.dt, i, info.n_timesteps, info.goal_set, cart_start, info.eef_fk), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x -> 0, true)]);
    ret_val
end

function jointvel_costfn(u::AbstractVector)
    sq_norm(u)
end

"""
Returns a vector of length n_timesteps of joint velocity costs
"""
function get_jointvel_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = GeneralCost{n,m}[];
    for i = 0:(info.n_timesteps - 2)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> jointvel_costfn(u), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x -> 0, true)]);
    ret_val
end

"""
Returns the visibility cost at a given timestep, defined as the angle
between the head-object axis and the head-eef axis

eef_pos: cartesian eef position
object_pos: cartesian object position
head_pos: cartesian head position
"""
function visibility_costfn(eef_pos::AbstractVector, object_pos::AbstractVector, head_pos::AbstractVector)
    # return 0;
    obj_axis = object_pos .- head_pos;
    eef_axis = eef_pos .- head_pos;
    # add a small epsilon to denom for numerical stability
    ret_val = acos(dot(eef_axis, obj_axis)/(norm(eef_axis)*norm(obj_axis) + 0.001));
    ret_val
end

"""
Returns a vector of length n_timesteps of visibility costs
"""
function get_visibility_costs(info::ComotoProblemInfo)
    #n::Int, m::Int, head_traj::AbstractMatrix, object_pos::AbstractVector, ee_posfn::Function
    n, m = info.state_dims, info.ctrl_dims
    ret_val = GeneralCost{n,m}[];
    for i = 1:(info.n_timesteps - 1)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> visibility_costfn(info.eef_fk(x), info.object_pos, info.head_traj[:,i]), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x -> visibility_costfn(info.eef_fk(x), info.object_pos, info.head_traj[:,end]), true)]);
    ret_val
end

"""
Return the distance between each robot joint and its closest human joint

cart_joints: 3xN_ROBOT_JOINTS array of cartesian robot joint positions
human_pos: 3xN_HUMAN_JOINTS array of human joint positions
"""
function jointwise_distance(cart_joints::AbstractArray, human_pos::AbstractArray)
    ret_val = zeros(size(cart_joints[2]))
    for rjoint_idx = 1:size(cart_joints)[2]
        diff_arr = human_pos .- cart_joints[:,rjoint_idx];
        dists = [norm(diff_arr[:,x]) for x=1:size(human_pos[2])]
        ret_val[rjoint_idx] = minimum(dists)
    end
    ret_val
end

"""
Return the distance cost for a given timestep

cart_joints: 3xN_ROBOT_JOINTS array of cartesian robot joint positions
human_pos: 3xN_HUMAN_JOINTS array of human joint positions
human_vars: 3x3xN_HUMAN_JOINTS array of human joint covariance matrices
"""
function distance_costfn(cart_joints::AbstractArray, human_pos::AbstractArray, human_vars::AbstractArray)
    n_robot_joints = size(cart_joints)[2]
    n_human_joints = size(human_pos)[2]
    
    cost_total = 0.0;
    for rjoint_idx = 1:n_robot_joints
        for hjoint_idx = 1:n_human_joints
            diff = abs.(cart_joints[:,rjoint_idx] - human_pos[:,hjoint_idx]);
            curr_cost_inv = diff'*inv(human_vars[:,:,hjoint_idx])*diff;
            cost_total += 1/curr_cost_inv;
        end
    end
    if isnan(cost_total)
        println("NaN in distance cost");
    end
    return cost_total;
end

"""
Returns a vector of length n_timesteps of distance costs
"""
function get_distance_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = GeneralCost{n,m}[];
    
    for i = 1:(info.n_timesteps - 1)
        curr_hpos = info.human_traj[:,:,i];
        curr_hvars = info.human_vars_traj[:,:,:,i];
        append!(ret_val, [GeneralCost{n,m}((x, u) -> distance_costfn(info.full_fk(x), curr_hpos, curr_hvars), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}((x) -> distance_costfn(info.full_fk(x), info.human_traj[:,:,end], info.human_vars_traj[:,:,:,end]), true)]);
    ret_val
end

"""
Returns the nominal cost at a timestep, defined as the square cartesian distance between
end effector positions of the nominal and actual trajectories

cart_eef: cartesian end effector position (actual)
goal_eef: cartesian end effector position (nominal)
"""
function nominal_costfn(cart_eef::AbstractVector, goal_eef::AbstractVector)
    ret_val = sq_norm(cart_eef .- goal_eef);
    if isnan(ret_val)
        println("NaN in nominal cost");
    end
    ret_val
end

"""
Returns a vector of length n_timesteps of nominal costs
"""
function get_nominal_costs(info::ComotoProblemInfo)
    n, m = info.state_dims, info.ctrl_dims
    ret_val = GeneralCost{n,m}[];

    nom_eef_traj = [info.eef_fk(info.nominal_traj[:, i]) for i=1:info.n_timesteps];
    for i = 1:(info.n_timesteps - 1)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> nominal_costfn(info.eef_fk(x), nom_eef_traj[i]), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x->nominal_costfn(info.eef_fk(x), nom_eef_traj[end]), true)]);
    ret_val
end

"""
Returns a vector of length n_timesteps of comoto costs (combination of legibility,
visibility, distance, nominal, and joint velocity costs). Costs are weighted
according to the weights vector.
Weights: leg, vis, dist, nom, vel
"""
function get_comoto_costs(info::ComotoProblemInfo, weights::AbstractVector)
    leg_costs = get_legibility_costs(info);
    vis_costs = get_visibility_costs(info);
    dist_costs = get_distance_costs(info);
    nom_costs = get_nominal_costs(info);
    vel_costs = get_jointvel_costs(info);
    return [CompoundCost([leg_costs[i], vis_costs[i], dist_costs[i], nom_costs[i], vel_costs[i]], leg_costs[i].is_terminal, info.ctrl_dims, info.state_dims, weights) for i=1:info.n_timesteps];
end

"""
Convenience function to construct comoto costs, add typical constraints for the iiwa,
    set up joint-space linear initial controls, and return the trajopt problem

model: robot model
weights: comoto weights
"""
function get_comoto_problem(model::TO.AbstractModel, info::ComotoProblemInfo, weights::AbstractVector)
    final_costs = get_comoto_costs(info, weights)
    n_timesteps = info.n_timesteps
    tf = (n_timesteps-1)*info.dt;
    obj = TO.Objective(final_costs);
    ctrl_linear = (info.joint_target - info.joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];

    cons = TO.ConstraintList(info.ctrl_dims, info.state_dims, n_timesteps);
    add_constraint!(cons, TO.GoalConstraint(info.joint_target), n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    add_constraint!(cons, PosEECons(info.ctrl_dims, info.ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,100,100], info.full_fk), 2:n_timesteps-1);

    prob = TO.Problem(model, obj, info.joint_target, tf, x0=info.joint_start, constraints=cons);
    initial_controls!(prob, U0);
    prob
end

"""
Convenience function to confirm start, move to trajectory start,
    confirm execution, and execute a trajectory through ROS
"""
function confirm_display_traj(solver::ALTROSolver, dt::Float64)
    println("Ready to move to start?")
    readline(stdin)
    move_to(TO.states(solver)[1], 4.0)
    println("Ready to dispatch?")
    readline(stdin)
    dispatch_trajectory(hcat(TO.states(solver)...), dt, 0.)
end
