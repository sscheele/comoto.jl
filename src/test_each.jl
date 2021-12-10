include("comoto.jl")

# note: object set/pos are already in julia space, not sim space
const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function main()
    model = VelCtrl_7dof(0)

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
    
    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        verbose_pn=true,
        iterations_inner=60,
        iterations_outer=15,
        penalty_initial=0.1,
        verbose=1,
    )

    i = 1;
    cost_names = ["leg", "vis", "dist", "nom"]
    while true
        weights = [0., 0., 0., 0., 1.];
        println("Enter ", cost_names[i], " cost")
        try
            my_weight = parse(Float64, readline(stdin));
            weights[i] = my_weight;
        catch
            println("Continuing...")
            i += 1;
            if i > 4
                println("Done!")
                break
            end
            continue;
        end
        prob = get_comoto_problem(model, prob_info, weights)

        solver = ALTROSolver(prob, opts);
        solve!(solver)

        confirm_display_traj(TO.states(solver), prob_info.dt*(prob_info.n_timesteps - 1), "IIWA", "means.csv");
    end
end

main()