include("comoto.jl")

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

test_pure_legibility()