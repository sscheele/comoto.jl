using RigidBodyDynamics;
using LinearAlgebra;

include("util.jl")

struct ComotoProblemInfo
    joint_tree::RigidBodyDynamics.Mechanism
    eef_fk::Function
    full_fk::Function
    ctrl_dims::Int
    state_dims::Int
    n_timesteps::Int
    dt::Float64
    object_pos::AbstractVector
    goal_set::AbstractMatrix
    human_traj::AbstractArray
    head_traj::AbstractArray
    human_vars_traj::AbstractArray
    joint_start::AbstractVector
    joint_target::AbstractVector
    nominal_traj::AbstractMatrix
end

mutable struct ComotoParameters
    joint_start::AbstractVector
    joint_target::AbstractVector
    n_timesteps::Int
    dt::Float64
    goal_set::AbstractMatrix
end

# struct KukaFKCacheLine
#     full_pos::AbstractMatrix
#     eef_pos::AbstractVector
# end

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

function get_kuka_probinfo(params::ComotoParameters, urdf_filepath::String="kuka.urdf", 
    means_filepath::String="means.csv", vars_filepath::String="vars.csv")
    
    kuka_tree = parse_urdf(urdf_filepath, remove_fixed_tree_joints=false)
    cache = StateCache(kuka_tree)
    end_effector_fn = get_kuka_ee_postition_fun(kuka_tree, cache);
    # jacobian_fn = get_kuka_jacobian_fun(kuka_tree);
    
    human_traj, head_traj, human_vars_traj = read_human_traj_files(means_filepath, vars_filepath);
    human_traj = resample_human_traj(human_traj, params.n_timesteps);
    head_traj = resample_human_traj(head_traj, params.n_timesteps);
    human_vars_traj = resample_human_traj(human_vars_traj, params.n_timesteps);

    nom_traj = get_nominal_traj(params.joint_start, params.joint_target, params.n_timesteps);

    ComotoProblemInfo(
        kuka_tree,
        end_effector_fn,
        function (x::AbstractVector{T}) where T
            kuka_full_fk(x, kuka_tree, cache)
        end,
        7,
        7,
        params.n_timesteps,
        params.dt,
        params.goal_set[:,1],
        params.goal_set,
        human_traj,
        head_traj,
        human_vars_traj,
        params.joint_start,
        params.joint_target,
        nom_traj
    )
end