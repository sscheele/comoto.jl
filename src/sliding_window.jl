include("comoto.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

const RESAMPLE_PERIOD = 2;
const RESAMPLE_HORIZON = 5;

function main()
    model = Kuka(0)

    joint_start = @SVector [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    # joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    n_timesteps = 20;
    dt = 0.25;

    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        dt,
        OBJECT_SET
    )
    base_prob_info = get_kuka_probinfo(params)

    weights = @SVector [2., 1.5, 2., 7., 0.1];

    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        iterations_inner=60,
        iterations_outer=15,
        penalty_initial=0.1,
        verbose=0,
    )

    subproblem_start = base_prob_info.joint_start
    t_total = 0;
    n_iter = 0;
    actual_traj = Array{Any}(undef, n_timesteps);
    for i = 1:RESAMPLE_PERIOD:(n_timesteps - RESAMPLE_HORIZON - RESAMPLE_PERIOD)
        t_start = Dates.now()
        subprob_info = ComotoProblemInfo(
            base_prob_info.joint_tree,
            base_prob_info.eef_fk,
            base_prob_info.full_fk,
            base_prob_info.ctrl_dims,
            base_prob_info.state_dims,
            RESAMPLE_HORIZON+1,
            base_prob_info.dt,
            OBJECT_POS,
            OBJECT_SET,
            base_prob_info.human_traj[:,:,i:i+RESAMPLE_HORIZON],
            base_prob_info.head_traj[:,i:i+RESAMPLE_HORIZON],
            base_prob_info.human_vars_traj[:,:,:,i:i+RESAMPLE_HORIZON],
            subproblem_start,
            joint_target,
            base_prob_info.nominal_traj[:,i:i+RESAMPLE_HORIZON]
        )
        prob = get_partial_comoto(model, subprob_info, weights);
        
        solver = ALTROSolver(prob, opts);
        solve!(solver)
        t_elapsed = Dates.now() - t_start;
        if i != 1
            t_total += Dates.value(t_elapsed)
        end
        
        actual_traj[i:i+RESAMPLE_HORIZON] = TO.states(solver);
        subproblem_start = TO.states(solver)[RESAMPLE_PERIOD+1];
        println("JS distance to goal: ", sq_norm(joint_target - subproblem_start))
        n_iter += 1
    end
    i = n_iter*RESAMPLE_PERIOD + 1
    final_prob_info = ComotoProblemInfo(
        base_prob_info.joint_tree,
        base_prob_info.eef_fk,
        base_prob_info.full_fk,
        base_prob_info.ctrl_dims,
        base_prob_info.state_dims,
        RESAMPLE_HORIZON+1,
        base_prob_info.dt,
        OBJECT_POS,
        OBJECT_SET,
        base_prob_info.human_traj[:,:,i:i+RESAMPLE_HORIZON],
        base_prob_info.head_traj[:,i:i+RESAMPLE_HORIZON],
        base_prob_info.human_vars_traj[:,:,:,i:i+RESAMPLE_HORIZON],
        subproblem_start,
        joint_target,
        base_prob_info.nominal_traj[:,i:i+RESAMPLE_HORIZON]
    )
    prob = get_comoto_problem(model, final_prob_info, weights)
    solver = ALTROSolver(prob, opts);
    solve!(solver)
    actual_traj[i:i+RESAMPLE_HORIZON] = TO.states(solver);

    println(actual_traj)

    println("Average time: ", t_total/(n_iter-1), " ms")

    confirm_display_traj(actual_traj, dt*(n_timesteps-1), "means.csv")
end

function get_partial_comoto(model::TO.AbstractModel, info::ComotoProblemInfo, weights::AbstractVector)
    final_costs = get_comoto_costs(info, weights)
    n_timesteps = info.n_timesteps
    tf = (n_timesteps-1)*info.dt;
    obj = TO.Objective(final_costs);
    ctrl_linear = (info.joint_target - info.joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];

    cons = TO.ConstraintList(info.ctrl_dims, info.state_dims, n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    add_constraint!(cons, PosEECons(info.ctrl_dims, info.ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,100,100], info.full_fk), 2:n_timesteps-1);

    prob = TO.Problem(model, obj, info.joint_target, tf, x0=info.joint_start, constraints=cons);
    initial_controls!(prob, U0);
    prob
end

function confirm_display_traj(solved_traj::AbstractArray, total_time::Float64, human_trajfile::String="")
    # TODO: change this and exec_human_traj.py to use total time (to avoid different dt's)
    println("Ready to move to start?")
    readline(stdin)
    move_to(solved_traj[1], 4.0)
    println("Ready to dispatch?")
    readline(stdin)
    @sync begin
        human_dt = total_time/(countlines(human_trajfile)-1);
        if human_trajfile != ""
            @async dispatch_human_trajectory(human_trajfile, human_dt);
        end
        robot_dt = total_time/(length(solved_traj)-1);
        @async dispatch_trajectory(hcat(solved_traj...), robot_dt, 0.);
    end
end

main()