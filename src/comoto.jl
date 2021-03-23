import Pkg; Pkg.activate("..");

using RobotDynamics, Rotations
using TrajectoryOptimization
using StaticArrays, LinearAlgebra
using RigidBodyDynamics;
using Altro;
import ForwardDiff;
import CSV;
using Dates;

include("ros_interface.jl")

const RBD = RigidBodyDynamics;
const TO = TrajectoryOptimization;

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function get_kuka_joint(kuka::Mechanism, jointname::String)
    ee_body = findbody(kuka, jointname)
    ee_point = Point3D(default_frame(ee_body),0.,0.,0.)
    return ee_body, ee_point
end

function kuka_full_fk(x::AbstractVector{T},kuka::Mechanism,statecache=StateCache(kuka)) where T
    kuka_joints = ["iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
        "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"]
    state_vec = T[];
    state = statecache[T];
    world = root_frame(kuka);
    nn = num_positions(kuka);

    RBD.set_configuration!(state, x[1:nn])
    idx = 1;
    for jointname in kuka_joints
        body, point = get_kuka_joint(kuka, jointname);
        append!(state_vec, RBD.transform(state, point, world).v);
        idx += 3;
    end
    reshape(state_vec, (3,:))
end

function get_kuka_jacobian_fun(kuka::Mechanism,statecache=StateCache(kuka))
    root_body = RigidBodyDynamics.findbody(kuka, "base");
    ee_body = RigidBodyDynamics.findbody(kuka, "iiwa_link_ee");
    joint_path = RigidBodyDynamics.path(kuka, root_body, ee_body);

    function kuka_jacobian(x::AbstractVector{T}) where T
        state = statecache[T]
        RBD.set_configuration!(state, x)
        RigidBodyDynamics.geometric_jacobian(state, joint_path)
    end
end

function get_kuka_ee_postition_fun(kuka::Mechanism,statecache=StateCache(kuka))
    ee_body, ee_point = get_kuka_joint(kuka, "iiwa_link_ee")
    world = root_frame(kuka)
    nn = num_positions(kuka)

    function ee_position(x::AbstractVector{T}) where T
        state = statecache[T]
        RBD.set_configuration!(state, x[1:nn])
        RBD.transform(state, ee_point, world).v
    end
end

struct Kuka <: TrajectoryOptimization.AbstractModel
    id::Int32
end

RobotDynamics.control_dim(::Kuka) = 7
RobotDynamics.state_dim(::Kuka) = 7

function RobotDynamics.dynamics(model::Kuka, x, u)
    u
end

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
    # println("State size: "*string(size(x)))
    xyz = cons.fk(x)
    ans = T[]

    curr_idx = 1;
    for i = 1:cons.n_joints
        joint_pos = xyz[:,i]
        append!(ans, cons.lower_bounds - joint_pos)
        append!(ans, joint_pos - cons.upper_bounds)
        curr_idx += 6;
    end

    # if any(isnan.(ans))
    #     println("NaN in constraints")
    # end
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
goalset: 3xN matrix possible goals - first element (goalset[:,1]) is "true goal"
start: initial cartesian position
"""
function legibility_costfn(x::AbstractVector, u::AbstractVector, dt::Float64, curr_timestep::Int, total_timesteps::Int, goalset::AbstractMatrix, start::AbstractVector, ee_fn::Function)
    # return 0;
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

n: state dim
m: control dim
goalset: 3xn set of goal candidates, true goal at goalset[:,1]
start: joint start position
"""
function get_legibility_costs(n::Int, m::Int, dt::Float64, total_timesteps::Int, goalset::AbstractMatrix, start::AbstractVector, ee_fn::Function)
    ret_val = GeneralCost{n,m}[];
    cart_start = ee_fn(start);
    for i = 0:(total_timesteps - 2)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> legibility_costfn(x, u, dt, i, total_timesteps, goalset, cart_start, ee_fn), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x -> 0, true)]);
    ret_val
end

function jointvel_costfn(u::AbstractVector)
    sq_norm(u)
end

function get_jointvel_costs(n::Int, m::Int, n_timesteps::Int)
    ret_val = GeneralCost{n,m}[];
    for i = 0:(n_timesteps - 2)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> jointvel_costfn(u), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x -> 0, true)]);
    ret_val
end

function visibility_costfn(eef_pos::AbstractVector, axis::AbstractVector)
    # return 0;
    ret_val = acos(dot(eef_pos, axis)/(norm(eef_pos)*norm(axis)));
    if isnan(ret_val)
        println("NaN in visibility cost");
    end
    ret_val
end

"""
Returns an array of length total_timesteps of visibility cost functions

n: state dim
m: control dim
head_traj: 3xNUM_TIMESTEPS matrix of head positions
eef_posfn: fk function (joint state) -> cartesian eef position
"""
function get_visibility_costs(n::Int, m::Int, head_traj::AbstractMatrix, object_pos::AbstractVector, ee_posfn::Function)
    ret_val = GeneralCost{n,m}[];
    n_timesteps = size(head_traj)[2]
    for i = 1:(n_timesteps - 1)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> visibility_costfn(ee_posfn(x), object_pos - head_traj[:,i]), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x -> visibility_costfn(ee_posfn(x), object_pos - head_traj[:,end]), true)]);
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
Return the minimum human-robot distance
"""
function distance(cart_joints::AbstractArray, human_pos::AbstractArray)
    
end

"""
Return the distance cost for a given timestep

cart_joints: 3xN_ROBOT_JOINTS array of cartesian robot joint positions
human_pos: 3xN_HUMAN_JOINTS array of human joint positions
human_vars: 3x3xN_HUMAN_JOINTS array of human joint covariance matrices
"""
function distance_costfn(cart_joints::AbstractArray, human_pos::AbstractArray, human_vars::AbstractArray)
    # return 0;
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
Return a list of length N_TIMESTEPS of distance costs

human_traj: 3xN_HUMAN_JOINTSxN_TIMESTEPS human cartesian trajectory
human_var_traj: 3x3xN_HUMAN_JOINTSxN_TIMESTEPS array of arrays of human joint covariance matrices
fk: forward kinematics function (joint state) -> list of joint cartesian positions
"""
function get_distance_costs(n::Int, m::Int, human_traj::AbstractArray, human_var_traj::AbstractArray, fk::Function)
    ret_val = GeneralCost{n,m}[];
    n_timesteps = size(human_traj)[3]
    for i = 1:(n_timesteps - 1)
        curr_hpos = human_traj[:,:,i];
        curr_hvars = human_var_traj[:,:,:,i];
        append!(ret_val, [GeneralCost{n,m}((x, u) -> distance_costfn(fk(x), curr_hpos, curr_hvars), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}((x) -> distance_costfn(fk(x), human_traj[:,:,end], human_var_traj[:,:,:,end]), true)]);
    ret_val
end

# function smoothness_costfn()

function nominal_costfn(cart_eef::AbstractVector, goal_eef::AbstractVector)
    ret_val = sq_norm(cart_eef .- goal_eef);
    if isnan(ret_val)
        println("NaN in nominal cost");
    end
    ret_val
end

"""
Return a list of length N_TIMESTEPS of nominal costs

nominal_traj: N_ROBOT_JOINTSxN_TIMESTEPS robot joint trajectory
eef_fk: Forward kinematics (joint state)->eef position
"""
function get_nominal_costs(n::Int, m::Int, nominal_traj::AbstractMatrix, eef_fk::Function)
    ret_val = GeneralCost{n,m}[];
    n_timesteps = size(nominal_traj)[2]

    nom_eef_traj = [eef_fk(nominal_traj[:, i]) for i=1:n_timesteps];
    for i = 1:(n_timesteps - 1)
        append!(ret_val, [GeneralCost{n,m}((x, u) -> nominal_costfn(eef_fk(x), nom_eef_traj[i]), false)]);
    end
    append!(ret_val, [GeneralCost{n,m}(x->nominal_costfn(eef_fk(x), nom_eef_traj[end]), true)]);
    ret_val
end

function get_nominal_traj(start::AbstractVector, goal::AbstractVector, n_timesteps::Int)
    @assert length(start) == length(goal);
    ret_val = zeros(length(start), n_timesteps);
    for i = 1:length(start)
        ret_val[i,:] = collect(range(start[i], goal[i], length=n_timesteps));
    end
    SMatrix{length(start), n_timesteps}(ret_val)
end

function confirm_display_traj(solver::ALTROSolver, dt::Float64)
    println("Ready to move to start?")
    readline(stdin)
    move_to(TO.states(solver)[1], 4.0)
    println("Ready to dispatch?")
    readline(stdin)
    dispatch_trajectory(hcat(TO.states(solver)...), dt, 0.)
end

function test_pure_legibility()
    model = Kuka(0)
    # size(model)

    kuka_tree = parse_urdf("kuka.urdf",remove_fixed_tree_joints=false)
    end_effector_fn = get_kuka_ee_postition_fun(kuka_tree);

    ctrl_dims, state_dims, dt = 7, 7, 0.25
    n_timesteps = 20
    joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    joint_start = @SVector [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]

    leg_costs = get_legibility_costs(ctrl_dims, state_dims, dt, n_timesteps, OBJECT_SET, joint_start, end_effector_fn);
    vel_costs = get_jointvel_costs(ctrl_dims, state_dims, n_timesteps);

    tf = (n_timesteps-1)*dt;
    ctrl_linear = (joint_target - joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];

    cons = TO.ConstraintList(ctrl_dims, state_dims, n_timesteps);
    add_constraint!(cons, TO.GoalConstraint(joint_target), n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(ctrl_dims, state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(ctrl_dims, state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    # add table constraints
    add_constraint!(cons, PosEECons(ctrl_dims, ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,100,100], x -> kuka_full_fk(x, kuka_tree)), 2:n_timesteps-1);

    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        # verbose_pn=true,
        iterations_inner=60,
        iterations_outer=15,
        penalty_initial=0.1,
        verbose=1
    )

    while true
        println("Enter leg weight")
        leg_weight = parse(Float64, readline(stdin))
        println("Enter velocity weight")
        vel_weight = parse(Float64, readline(stdin))
        weights = @SVector [leg_weight, vel_weight];
        final_costs = [CompoundCost([leg_costs[i], vel_costs[i]], leg_costs[i].is_terminal, ctrl_dims, state_dims, weights) for i=1:n_timesteps];
        obj = TO.Objective(final_costs);
        prob = TO.Problem(model, obj, joint_target, tf, x0=joint_start, constraints=cons);
        initial_controls!(prob, U0);

        solver = ALTROSolver(prob, opts);
        solve!(solver);
        println(TO.states(solver))
        println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)
        confirm_display_traj(solver, dt)
    end
end

function main()
    model = Kuka(0)
    # size(model)

    kuka_tree = parse_urdf("kuka.urdf",remove_fixed_tree_joints=false)
    end_effector_fn = get_kuka_ee_postition_fun(kuka_tree);
    jacobian_fn = get_kuka_jacobian_fun(kuka_tree);
    # should be about [0, 0, 1.306]
    joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    # @show kuka_full_fk([0, 0, 0, 0, 0, 0, 0], kuka_tree)
    @show kuka_full_fk(joint_target, kuka_tree);
    ctrl_dims = 7;
    state_dims = 7;
    dt = 0.25;
    n_timesteps = 20;
    N_HUMAN_JOINTS = 11;
    
    # read in human trajectory means
    means_reader = CSV.File("means.csv");
    human_traj = zeros(3,N_HUMAN_JOINTS,n_timesteps)
    head_traj = zeros(3, n_timesteps)
    curr_timestep = 1;
    head_symbols = CSV.Symbol.(["headx", "heady", "headz"])
    for row in means_reader
        i = 1;
        while i < N_HUMAN_JOINTS
            human_traj[:,i,curr_timestep] .= [row[3*(i-1) + x] for x=1:3];
            i += 1;
        end
        head_traj[:,curr_timestep] = [row[s] for s in head_symbols];

        curr_timestep += 1;
    end
    human_traj = SArray{Tuple{3,N_HUMAN_JOINTS,n_timesteps}}(human_traj);
    head_traj = SArray{Tuple{3,n_timesteps}}(head_traj);

    # read in human trajectory vars
    vars_reader = CSV.File("vars.csv", header=false);
    human_vars_traj = zeros(3,3,N_HUMAN_JOINTS,n_timesteps);
    curr_timestep = 1;
    for row in vars_reader
        for curr_joint = 1:N_HUMAN_JOINTS
            for matr_row = 1:3
                idx_start = (curr_joint-1)*9 + (matr_row-1)*3
                human_vars_traj[matr_row,:,curr_joint,curr_timestep] = [row[idx_start+i] for i=1:3];
            end
        end
        curr_timestep += 1
    end

    human_vars_traj = SArray{Tuple{3,3,N_HUMAN_JOINTS,n_timesteps}}(human_vars_traj);

    joint_start = @SVector [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    # joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case

    # uncomment these lines to switch to single-timestep trajectories
    # n_timesteps = 3;
    # human_traj = human_traj[:,:,[1,10,20]];
    # human_vars_traj = human_vars_traj[:,:,:,[1,10,20]];
    # head_traj = head_traj[:,[1,10,20]];
    # dt = 2.5;

    nom_traj = get_nominal_traj(joint_start, joint_target, n_timesteps);

    leg_costs = get_legibility_costs(ctrl_dims, state_dims, dt, n_timesteps, OBJECT_SET, joint_start, end_effector_fn);
    vis_costs = get_visibility_costs(ctrl_dims, state_dims, head_traj, OBJECT_POS, end_effector_fn);
    dist_costs = get_distance_costs(ctrl_dims, state_dims, human_traj, human_vars_traj, x -> kuka_full_fk(x, kuka_tree));
    nom_costs = get_nominal_costs(ctrl_dims, state_dims, nom_traj, end_effector_fn);
    vel_costs = get_jointvel_costs(ctrl_dims, state_dims, n_timesteps);
    weights = @SVector [2., 0.0015, 2., 0.1, 0.1];
    final_costs = [CompoundCost([leg_costs[i], vis_costs[i], dist_costs[i], nom_costs[i], vel_costs[i]], leg_costs[i].is_terminal, ctrl_dims, state_dims, weights) for i=1:n_timesteps];


    tf = (n_timesteps-1)*dt;
    obj = TO.Objective(final_costs);
    ctrl_linear = (joint_target - joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];

    cons = TO.ConstraintList(ctrl_dims, state_dims, n_timesteps);
    add_constraint!(cons, TO.GoalConstraint(joint_target), n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(ctrl_dims, state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(ctrl_dims, state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    prob = TO.Problem(model, obj, joint_target, tf, x0=joint_start, constraints=cons);
    initial_controls!(prob, U0);

    println("Beginning to attempt solution");
    # rollout!(prob);

    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        verbose_pn=true,
        iterations_inner=60,
        iterations_outer=15,
        penalty_initial=0.1,
        verbose=1,
    )

    solver = ALTROSolver(prob, opts);
    solve!(solver)
    println("Cost: ", cost(solver))
    println("States: ", TO.states(solver))
    println("Controls: ", TO.controls(solver))
    println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    println("Violated control constraints: ", any(x->any(y->y<-5||y>5, x), TO.controls(solver)))
    println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)

    for _ = 1:10
        t_start = Dates.now()
        newsolver = ALTROSolver(prob, opts);
        solve!(newsolver)
        println("Time to solve: ", Dates.now() - t_start)
        println(TO.states(newsolver))
    end
end

# main()
test_pure_legibility()