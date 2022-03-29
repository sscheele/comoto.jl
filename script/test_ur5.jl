import Pkg; Pkg.activate("..")
# include("iiwa_problem_info.jl")
include("../src/problem_info.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function main()
    model = ManipVelCtrl{6}(0)

    joint_start = @SVector [-2.33, 4.9, 2.2, 0.7, 1.7, 5.24]; # (-.106, -.255, .135)
    joint_target = @SVector [-3.86, 5.43, 1.3, 1.06, 1.7, 5.24]; # on ur5, this is (-.47, .28, .15)
    n_timesteps = 20;
    dt = 0.25;

    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        dt,
        OBJECT_SET
    )
    # prob_info = get_kuka_probinfo(params)
    # prob_info = get_jaco_probinfo(params)
    # TODO: replace joint names with link names
    ur5_info = RobotInfo(
        ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"],
        "ur5.urdf",
        "tool0",
        6
    )
    prob_info = get_probinfo(params, ur5_info);
    
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
    println("States: ", TO.states(solver))
    println("Controls: ", TO.controls(solver))
    println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    println("Violated control constraints: ", any(x->any(y->y<-5||y>5, x), TO.controls(solver)))
    println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)
end

main()