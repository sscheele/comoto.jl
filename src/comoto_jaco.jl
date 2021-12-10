include("comoto.jl")
include("util.jl")

function jaco_full_fk(x::AbstractVector{T},jaco::Mechanism,statecache=StateCache(jaco)) where T
    jaco_joints = ["j2s7s300_link_1", "j2s7s300_link_2", "j2s7s300_link_3", "j2s7s300_link_4", "j2s7s300_link_5", "j2s7s300_link_6", "j2s7s300_link_7"]
    state_vec = T[];
    state = statecache[T];
    world = root_frame(jaco);
    nn = num_positions(jaco);

    RBD.set_configuration!(state, x[1:nn])
    idx = 1;
    for jointname in jaco_joints
        body, point = get_joint(jaco, jointname);
        append!(state_vec, RBD.transform(state, point, world).v);
        idx += 3;
    end
    reshape(state_vec, (3,:))
end

function get_jaco_jacobian_fun(jaco::Mechanism,statecache=StateCache(jaco))
    root_body = RigidBodyDynamics.findbody(jaco, "j2s7s300_link_base");
    ee_body = RigidBodyDynamics.findbody(jaco, "j2s7s300_ee_link");
    joint_path = RigidBodyDynamics.path(jaco, root_body, ee_body);

    function jaco_jacobian(x::AbstractVector{T}) where T
        state = statecache[T]
        RBD.set_configuration!(state, x)
        RigidBodyDynamics.geometric_jacobian(state, joint_path)
    end
end

function get_jaco_ee_postition_fun(jaco::Mechanism,statecache=StateCache(jaco))
    ee_body, ee_point = get_joint(jaco, "j2s7s300_ee_link")
    world = root_frame(jaco)
    nn = num_positions(jaco)

    function ee_position(x::AbstractVector{T}) where T
        state = statecache[T]
        RBD.set_configuration!(state, x[1:nn])
        RBD.transform(state, ee_point, world).v
    end
end

function get_jaco_ee_function(urdf_filepath::String="jaco_urdf.urdf")
    jaco_tree = parse_urdf(urdf_filepath, remove_fixed_tree_joints=false)
    cache = StateCache(jaco_tree)
    end_effector_fn = get_jaco_ee_postition_fun(jaco_tree, cache);
    end_effector_fn
end

function get_jaco_probinfo(params::ComotoParameters, urdf_filepath::String="jaco_urdf.urdf", 
    means_filepath::String="means.csv", vars_filepath::String="vars.csv")
    
    jaco_tree = parse_urdf(urdf_filepath, remove_fixed_tree_joints=false)
    cache = StateCache(jaco_tree)
    end_effector_fn = get_jaco_ee_postition_fun(jaco_tree, cache);
    # jacobian_fn = get_kuka_jacobian_fun(kuka_tree);
    
    human_traj, head_traj, human_vars_traj, human_goal = read_human_traj_files(means_filepath, vars_filepath, offset=[0.5, 0., -0.75]);
    human_traj = resample_human_traj(human_traj, params.n_timesteps);
    head_traj = resample_human_traj(head_traj, params.n_timesteps);
    human_vars_traj = resample_human_traj(human_vars_traj, params.n_timesteps);

    nom_traj = get_nominal_traj(params.joint_start, params.joint_target, params.n_timesteps);

    ComotoProblemInfo(
        jaco_tree,
        end_effector_fn,
        function (x::AbstractVector{T}) where T
            jaco_full_fk(x, jaco_tree, cache)
        end,
        7,
        7,
        params.n_timesteps,
        params.dt,
        params.goal_set[:,1],
        params.goal_set,
        human_goal,
        human_traj,
        head_traj,
        human_vars_traj,
        params.joint_start,
        params.joint_target,
        nom_traj
    )
end