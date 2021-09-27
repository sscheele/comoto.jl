include("comoto_jaco.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function main(urdf_filepath::String="jaco_urdf.urdf")
    model = VelCtrl_7dof(0)

    joint_start = @SVector [-0.7240388673767146, -0.34790398102066433, 2.8303899987665897, -2.54032606205873, 1.3329587647643253, 2.7596249683074614, 0.850582268802067]
    # join_pos =  @SVector  [2.5203525710892904e-06, -5.114979239362327e-05, 2.9007499149151625, 0.00023988308964195681, 1.2973341908758345, -2.0768131629122526, 1.4003935456135341, 8.866465064372164e-07]
    # joint_pos_1 = @SVector [-1.38, 3.14, -0.44, 1.88, -1.01, 1.4, 0.63]
    # joint_pos_2 = @SVector [-0.69, 3.65, -0.94, 2.57, -0.25, 3.62, 0.63]
    # joint_pos_3 = @SVector [-2.51, 1.7, 0.25, 2.15, -0.57, 2.22, 0.63]
    joint_target = @SVector [-1.38, 3.14, -0.44, 1.88, -1.01, 1.4, 0.63];
    # # joint_target = [0.04506347261090404, 1.029660363493563, -0.0563325987175789, -1.8024937659056217, 0.14645022654203643, 0.3406148976556631, -0.12291455548612884] #near reaching case
    # joint_target = @SVector [-0.3902233335085379, 1.7501020413442578, 0.8403277122861033, -0.22924505085794067, 2.8506926562622024, -1.417026666483551, -0.35668663982214976] #far reaching case
    
    n_timesteps = 20;
    dt = 0.25;

    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        dt,
        OBJECT_SET
    )
    prob_info = get_jaco_probinfo(params)
    
    weights = @SVector [2., 0.0015, 2., 0.1, 0.1];
    prob = get_comoto_problem(model, prob_info, weights)

    println("Beginning to attempt solution");

    opts = SolverOptions(
        penalty_scaling=10.,
        active_set_tolerance_pn=0.01,
        verbose_pn=true,
        iterations_inner=60,
        iterations_outer=15,
        penalty_initial=0.1,
        verbose=1,
    )

    solver = ALTROSolver(prob, opts);
    solve!(solver)
    println("Cost: ", cost(solver))
    # println("States: ", TO.states(solver))
    println("Controls: ", TO.controls(solver))
    println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    println("Violated control constraints: ", any(x->any(y->y<-10||y>10, x), TO.controls(solver)))
    println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)

    # confirm_display_traj(solver, dt*(n_timesteps - 1), "means.csv")
end

main()
