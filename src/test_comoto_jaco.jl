include("comoto_jaco.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([-0.07, -0.41, 0, -0.34, -0.70, 0], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function main(urdf_filepath::String="jaco_urdf.urdf")
    model = VelCtrl_7dof(0)

    # joint_start = @SVector [4.93928, 2.8396507, 6.28319, 0.76043996, 4.628979695, 4.494728969, 5.028974253]
    #joint_start = @SVector [3.298604109345377, 3.9015456443520793, 6.929810939862968, 0.9327826247075317, 3.1043060014837027, 4.336929872002593, 5.031395198927613]
    # joint_target = @SVector [5.046514812, 4.111088146,6.819123561, 0.9933942863, 4.413815505, 4.151859038, 5.491224706];
    #joint_start = @SVector [1.0707005470277922, 2.1676239097290497, 2.8747500376343607, 1.7463839969440553, 4.236838690857491, 2.63071926062822, 4.499131562406633];
    
    # Fixed joint starts
    # Home
    #joint_start = @SVector [1.7994737138006214, 3.4414297049224354, 3.1433806999456726, 0.7578406379646626, 4.634932537976389, 4.492826794841979, 5.0256560867784925];
    # Left
    #joint_start = @SVector [6.275706085574208, 3.5177979797093295, 6.284589858338494, 1.2498775591151325, 4.631694666723071, 4.492623861966915, 5.031556586489672];
    # Right
    joint_start = @SVector [1.0707005470277922, 2.1676239097290497, 2.8747500376343607, 1.7463839969440553, 4.236838690857491, 2.63071926062822, 4.499131562406633];
    #joint_start = @SVector [3.298604109345377, 3.9015456443520793, 6.929810939862968, 0.9327826247075317, 3.1043060014837027, 4.336929872002593, 5.031395198927613]; #from right side
    
    
    # Sampled joint targets
    # joint_target = @SVector [2.557013614299187, 2.344602815231666, 2.917716679713372, 1.2569398628518593, 4.908725738060819, 3.0464895731664097, 4.876310806536811];
    # joint_target = @SVector [2.1725037529524562, 2.1544880008102885, 3.2597142342630585, 1.6655121835957825, 4.778336838458322, 3.1676186616587017, 5.014504366029895];
    joint_target = @SVector [2.0466712556589757, 1.9449580762519607, 2.9728821974816, 2.0044336539283734, 4.761461451894926, 2.975863073689726, 5.005376115077539];
    n_timesteps = 20;
    dt = 0.25;

    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        dt,
        OBJECT_SET
    )
    prob_info = get_jaco_probinfo(params, "jaco_urdf.urdf", "human_traj_data/means_3.csv", "human_traj_data/vars_ol.csv")
    end_effector_fn = get_jaco_ee_function("jaco_urdf.urdf")
    
    # weights = @SVector [0., 0, 0, 1, 0];
    # prob = get_comoto_problem(model, prob_info, weights)

    # println("Beginning to attempt vanilla solution");

    # opts = SolverOptions(
    #     penalty_scaling=10.,
    #     active_set_tolerance_pn=0.01,
    #     verbose_pn=true,
    #     iterations_inner=60,
    #     iterations_outer=15,
    #     penalty_initial=0.1,
    #     verbose=1,
    # )

    # # solver = ALTROSolver(prob, opts);
    # # solve!(solver)
    # println("Cost: ", cost(solver))
    # # println("States: ", TO.states(solver))
    # println("Controls: ", TO.controls(solver))
    # println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    # println("Violated control constraints: ", any(x->any(y->y<-10||y>10, x), TO.controls(solver)))
    # println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)


    # confirm_display_traj(solver, "JACO", dt*(n_timesteps - 1), "human_traj_data/means.csv")
    
    weights = @SVector [1., 0.0015, 5., 0.1, 0.1];
    prob = get_comoto_problem(model, prob_info, weights)

    println("Beginning to attempt real solution");
    #move_to(joint_start, 4.0, "JACO")

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
    nominal_trajectory = [SVector{7}(prob_info.nominal_traj[:,t]) for t=1:n_timesteps]
    touch("traj_nominal.txt")
    open("traj_nominal.txt", "w") do file
        for angular in nominal_trajectory
            write(file, string(end_effector_fn(angular)))
            write(file, ",\n")
        end
    end
    solve!(solver)
    println("Cost: ", cost(solver))
    # println("States: ", TO.states(solver))
    println("Controls: ", TO.controls(solver))
    println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    println("Violated control constraints: ", any(x->any(y->y<-10||y>10, x), TO.controls(solver)))
    println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)
    
    # println("Executing Nominal Trajectory")
    # confirm_display_traj([SVector{7}(prob_info.nominal_traj[:,t]) for t=1:n_timesteps], "JACO", dt*(n_timesteps - 1), "human_traj_data/means.csv")
    println("Executing CoMOTO Trajectory")
    confirm_display_traj(TO.states(solver), "JACO", dt*(n_timesteps - 1), "human_traj_data/means_3.csv")
end

main()
