include("comoto.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];
const N_SAMPLES = 20;

function main()
    model = VelCtrl_7dof(0)

    joint_start = @SVector [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    # joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        iterations_inner=60,
        iterations_outer=15,
        penalty_initial=0.1
    )
    df = 5.0;

    for n_timesteps = 3:25
        dt = df/(n_timesteps-1)
        println(n_timesteps, " timesteps: ")

        params = ComotoParameters(
            joint_start,
            joint_target,
            n_timesteps,
            dt,
            OBJECT_SET
        )
        
        weights = @SVector [2., 0.0015, 2., 0.1, 0.1];

        total_time = 0
        for i = 1:(N_SAMPLES+1)
            prob_info = get_kuka_probinfo(params)
            prob = get_comoto_problem(model, prob_info, weights)
            t_start = Dates.now()
            newsolver = ALTROSolver(prob, opts);
            solve!(newsolver)
            elapsed = Dates.now() - t_start;
            # println("Time to solve: ", elapsed);
            if i != 1
                total_time += Dates.value(elapsed)
            end
        end
        println("Average time: ", total_time/N_SAMPLES, "ms");
        println("************************************")
    end

    
end

main()