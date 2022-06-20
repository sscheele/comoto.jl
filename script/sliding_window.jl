import Pkg; Pkg.activate("..");
import FiniteDiff;
include("../src/problem_info.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

const RESAMPLE_PERIOD = 2;
const RESAMPLE_HORIZON = 5;

function main(sliding_weights::AbstractVector{Float64})
    model = ManipVelCtrl{7}(0)

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
    kuka_joints = ["iiwa_link_1", "iiwa_link_2", "iiwa_link_3",
        "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"]
    kuka_info = RobotInfo(
        kuka_joints,
        "kuka.urdf",
        "iiwa_link_ee",
        7
    )
    base_prob_info = get_probinfo(params, kuka_info, "human_trajs/669-means-fmt.csv", 
        "human_trajs/669-vars-fmt.csv")

    # todo: cost_tolerance (default: 1e-4), cost_tolerance_intermediate (1e-4)
    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        iterations_inner=60,
        iterations_outer=5,
        penalty_initial=0.01,
        verbose=0,
        bp_reg=true,
        bp_reg_initial=0.01,
        cost_tolerance_intermediate=1e-1,
        # projected_newton_tolerance=1e-2,
        # ρ_chol=0.1,
        constraint_tolerance=1e-2
    )

    subproblem_start = base_prob_info.joint_start
    t_total = 0;
    n_iter = 0;
    actual_traj = Array{Any}(undef, n_timesteps);

    curr_prob_info = base_prob_info;
    tf = (RESAMPLE_HORIZON - 1)*curr_prob_info.dt;
    ctrl_linear = (joint_target - joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];
    prev_ctrls = U0;
    last_solved_posn_ts = 0;
    for i = 1:RESAMPLE_PERIOD:(n_timesteps - RESAMPLE_HORIZON)
        t_start = Dates.now()

        u_nom = (joint_target - subproblem_start)/(n_timesteps - i);
        nom_traj = zeros(kuka_info.dof, RESAMPLE_HORIZON+1);
        nom_traj[:,1] = subproblem_start;
        for i=1:RESAMPLE_HORIZON
            nom_traj[:,i+1] = nom_traj[:,i] + u_nom;
        end
        
        subprob_info = ComotoProblemInfo(
            curr_prob_info.joint_tree,
            curr_prob_info.eef_fk,
            curr_prob_info.full_fk,
            curr_prob_info.ctrl_dims,
            curr_prob_info.state_dims,
            RESAMPLE_HORIZON+1,
            curr_prob_info.dt,
            OBJECT_POS,
            OBJECT_SET,
            base_prob_info.human_goal,
            curr_prob_info.human_traj[:,:,i:i+RESAMPLE_HORIZON],
            curr_prob_info.head_traj[:,i:i+RESAMPLE_HORIZON],
            curr_prob_info.human_vars_traj[:,:,:,i:i+RESAMPLE_HORIZON],
            subproblem_start,
            joint_target,
            nom_traj #curr_prob_info.nominal_traj[:,i:i+RESAMPLE_HORIZON]
        )
        
        prob = get_partial_comoto(model, subprob_info, sliding_weights, prev_ctrls);

        solver = ALTROSolver(prob, opts);
        solve!(solver)
        t_elapsed = Dates.now() - t_start;
        if i != 1
            t_total += Dates.value(t_elapsed)
        end
        
        actual_traj[i:i+RESAMPLE_HORIZON] = TO.states(solver);
        last_solved_posn_ts = i+RESAMPLE_HORIZON;
        subproblem_start = TO.states(solver)[RESAMPLE_PERIOD+1];
        println("JS distance to goal: ", sq_norm(joint_target - subproblem_start))
        prev_ctrls = TO.controls(solver);
        n_iter += 1
    end
    
    u_end = (joint_target - actual_traj[last_solved_posn_ts])/(n_timesteps - last_solved_posn_ts);
    for i=(last_solved_posn_ts+1):length(actual_traj)
        actual_traj[i] = actual_traj[i-1]+u_end;
    end

    println(actual_traj)

    println("Average time: ", t_total/n_iter, " ms")

    confirm_display_traj(actual_traj, dt*(n_timesteps-1), "human_trajs/669-means-fmt.csv")
end

function get_partial_comoto(model::TO.AbstractModel, info::ComotoProblemInfo, weights::AbstractVector, U0::AbstractVector)
    final_costs = get_sliding_costs(info, weights)
    n_timesteps = info.n_timesteps
    tf = (n_timesteps-1)*info.dt;
    obj = TO.Objective(final_costs);
    

    cons = TO.ConstraintList(info.ctrl_dims, info.state_dims, n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(info.ctrl_dims, info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    add_constraint!(cons, PosEECons(info.ctrl_dims, info.ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,100,100], info.full_fk), 2:n_timesteps-1);

    prob = TO.Problem(model, obj, info.joint_start, tf, xf=info.joint_target, constraints=cons);
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

const RESAMPLE_HORIZON = 5;
const sliding_weights = @SVector [2., 1.5, 1.5, 20., 0.1, 1.5];
main(sliding_weights)

# while true
#     print("Enter to begin")
#     readline(stdin)
#     main()
# end