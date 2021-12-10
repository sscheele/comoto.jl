using RigidBodyDynamics;
using LinearAlgebra;

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

mutable struct ComotoParameters
    joint_start::AbstractVector
    joint_target::AbstractVector
    n_timesteps::Int
    dt::Float64
    goal_set::AbstractMatrix
end

function get_joint(kuka::Mechanism, jointname::String)
    ee_body = findbody(kuka, jointname)
    ee_point = Point3D(default_frame(ee_body),0.,0.,0.)
    return ee_body, ee_point
end

