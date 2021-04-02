include("comoto.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function test_pure_legibility()
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
    prob_info = get_kuka_probinfo(params)

    leg_costs = get_legibility_costs(prob_info);
    vel_costs = get_jointvel_costs(prob_info);

    tf = (n_timesteps-1)*dt;
    ctrl_linear = (joint_target - joint_start)/tf;
    U0 = [ctrl_linear for _ in 1:(n_timesteps-1)];

    cons = TO.ConstraintList(prob_info.ctrl_dims, prob_info.state_dims, n_timesteps);
    add_constraint!(cons, TO.GoalConstraint(joint_target), n_timesteps);
    add_constraint!(cons, TO.BoundConstraint(prob_info.ctrl_dims, prob_info.state_dims, u_min=-10, u_max=10), 1:n_timesteps-1);
    # cannot constrain final timestep twice
    add_constraint!(cons, TO.BoundConstraint(prob_info.ctrl_dims, prob_info.state_dims, x_min=-2π, x_max=2π), 1:n_timesteps-1);
    # add table constraints
    add_constraint!(cons, PosEECons(prob_info.ctrl_dims, prob_info.ctrl_dims, SA_F64[-100, -100, 0], SA_F64[100,100,100], prob_info.full_fk), 2:n_timesteps-1);

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
        final_costs = [CompoundCost([leg_costs[i], vel_costs[i]], leg_costs[i].is_terminal, prob_info.ctrl_dims, prob_info.state_dims, weights) for i=1:n_timesteps];
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

test_pure_legibility()