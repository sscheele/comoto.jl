"ComotoProblemInfo contains all neccessary information to compute all comoto costs"
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
    human_goal::AbstractVector
    human_traj::AbstractArray
    head_traj::AbstractArray
    human_vars_traj::AbstractArray
    joint_start::AbstractVector
    joint_target::AbstractVector
    nominal_traj::AbstractMatrix
end

"RobotInfo contains a robot specification, including joint names, urdf path,
end effector name, and degrees of freedom"
struct RobotInfo
    jnames::AbstractArray{String};
    urdf_path::String;
    eef_name::String;
    dof::Int;
end

"ComotoParameters are the user-defined paramters for a comoto problem"
mutable struct ComotoParameters
    joint_start::AbstractVector
    joint_target::AbstractVector
    n_timesteps::Int
    dt::Float64
    goal_set::AbstractMatrix
end