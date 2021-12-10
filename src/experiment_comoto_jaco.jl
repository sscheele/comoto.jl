include("comoto_jaco.jl")

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([-0.07, -0.41, 0, -0.34, -0.70, 0], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

function main(urdf_filepath::String="jaco_urdf.urdf")
    model = VelCtrl_7dof(0)
    vars_path = "human_traj_data/vars_ol.csv";

    # Joint Positions
    # A
    joint_A = @SVector [2.618481236470979, 2.1781692286974366, 3.988977222939381, 1.5927893768670434, 5.134235294949895, 4.207676277972796, 5.233939785104718];
    # B
    joint_B = @SVector [1.013891126497685, 2.0627883071019717, 4.007396976618914, 1.6509502850713385, 4.7400064935210295, 3.980682008776002, 4.057354949191601];
    # C
    joint_C = @SVector [0.7210472032924549, 1.8652933406677663, 2.874693578619251, 2.296507723841489, 4.268829647135959, 2.599898497332733, 4.543614874726444];
    # D
    joint_D = @SVector [0.3926770107511901, 2.3330297823986177, 2.893125316523689, 1.4296099068687818, 3.1891255516738544, 2.633823174878707, 4.7752651484570245];
    # E
    joint_E = @SVector [2.0466712556589757, 1.9449580762519607, 2.9728821974816, 2.0044336539283734, 4.761461451894926, 2.975863073689726, 5.005376115077539];

    # White Calculator - Calculator 1
    # means_path ="human_traj_data/means_201.csv";
    # means_path ="human_traj_data/means_201_62.csv";

    # (A, D, 1)
    # joint_start = joint_A;
    # joint_target = joint_D;
    # file_ending = "1_A_D_62"

    # Pink Calculator - Calculator 2
    # means_path ="human_traj_data/means_101.csv";
    # means_path ="human_traj_data/means_101_52.csv";

    # (D, E, 2)
    # joint_start = joint_D;
    # joint_target = joint_E;
    # file_ending = "2_D_E"

    # (E, C, 2)
    # joint_start = joint_E;
    # joint_target = joint_C;
    # file_ending = "2_E_C"


    # (A, B, 2)
    # joint_start = joint_A;
    # joint_target = joint_B;
    # file_ending = "2_A_B"
    
    # (E, B, 2)
    # joint_start = joint_E;
    # joint_target = joint_B;
    # file_ending = "2_E_B"


    # Black Calculator - Calculator 3
    # means_path ="human_traj_data/means_203.csv";
    # means_path ="human_traj_data/means_203_62.csv";
    # means_path ="human_traj_data/means_203_52.csv";


    # (A, C, 3)
    # joint_start = joint_A;
    # joint_target = joint_C;
    # file_ending = "3_A_C_52" 
    # 52 is amazing for this


    # (D, B, 3)
    # joint_start = joint_D;
    # joint_target = joint_B;
    # file_ending = "3_D_B_52"


     # (C, B, 3)
    # joint_start = joint_C;
    # joint_target = joint_B;
    # file_ending = "3_C_B_52"

    # Green Calculator - Calculator 4
    # means_path ="human_traj_data/means_204.csv";
    # means_path ="human_traj_data/means_204_52.csv";


    # (D, B, 4)
    # joint_start = joint_D;
    # joint_target = joint_B;
    # file_ending = "4_D_B_52"


    # (D, E, 4)
    # joint_start = joint_D;
    # joint_target = joint_E;
    # file_ending = "4_D_E_52"


    n_timesteps = 10;
    dt = 0.25;

    params = ComotoParameters(
        joint_start,
        joint_target,
        n_timesteps,
        dt,
        OBJECT_SET
    )
    prob_info = get_jaco_probinfo(params, "jaco_urdf.urdf", means_path, vars_path)
    end_effector_fn = get_jaco_ee_function("jaco_urdf.urdf")

    
    weights = @SVector [0, 0, 15., 0, 0.1]; #@SVector [1., 1, 15., 0.1, 0.1];
    prob = get_comoto_problem(model, prob_info, weights)

    println("Beginning to attempt real solution");

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
    nominal_file = string("traj_nominal_", file_ending, ".txt");
    touch(nominal_file)
    open(nominal_file, "w") do file
        for angular in nominal_trajectory
            write(file, string(angular)[2:end-1])
            write(file, "\n")
        end
        write(file, "0.25, 0.0")
    end
    println("Done Printing Nominal")
    solve!(solver)
    println("Cost: ", cost(solver))
    # println("States: ", TO.states(solver))
    println("Controls: ", TO.controls(solver))
    println("Violated joint constraints: ", any(x->any(y->y<-2π||y>2π, x), TO.states(solver)))
    println("Violated control constraints: ", any(x->any(y->y<-10||y>10, x), TO.controls(solver)))
    println("Reaches goal: ", sq_norm(joint_target - TO.states(solver)[end]) < 0.01)
    
    trajectory = TO.states(solver)
    controls = TO.controls(solver)
    distance_cost_fns = get_distance_costs(prob_info)
    legibility_cost_fns = get_legibility_costs(prob_info)
    visibility_cost_fns = get_visibility_costs(prob_info)
    nominal_cost_fns = get_nominal_costs(prob_info)
    jointvel_cost_fns = get_jointvel_costs(prob_info)

    println("HOLAAA")
    touch("distance.txt")
    open("distance.txt", "w") do file
        for i =1:length(trajectory)
            const_fn = distance_cost_fns[i]
            x = trajectory[i]
    
            if const_fn.is_terminal
                write(file, string(weights[3].*TO.stage_cost(const_fn, x)))
                write(file, "\n")
            else
                write(file, string(weights[3].*TO.stage_cost(const_fn, x, controls[i])))
                write(file, ", ")
            end
        end    
    end


    touch("legibility.txt")
    open("legibility.txt", "w") do file
        for i =1:length(trajectory)
            const_fn = legibility_cost_fns[i]
            x = trajectory[i]
    
            if const_fn.is_terminal
                write(file, string(weights[1].*TO.stage_cost(const_fn, x)))
                write(file, "\n")
            else
                write(file, string(weights[1].*TO.stage_cost(const_fn, x, controls[i])))
                write(file, ", ")
            end
        end    
    end


    touch("visibility.txt")
    open("visibility.txt", "w") do file
        for i =1:length(trajectory)
            const_fn = visibility_cost_fns[i]
            x = trajectory[i]
    
            if const_fn.is_terminal
                write(file, string(weights[2].*TO.stage_cost(const_fn, x)))
                write(file, "\n")
            else
                write(file, string(weights[2].*TO.stage_cost(const_fn, x, controls[i])))
                write(file, ", ")
            end
        end    
    end

    touch("nominal.txt")
    open("nominal.txt", "w") do file
        for i =1:length(trajectory)
            const_fn = nominal_cost_fns[i]
            x = trajectory[i]
    
            if const_fn.is_terminal
                write(file, string(weights[4].*TO.stage_cost(const_fn, x)))
                write(file, "\n")
            else
                write(file, string(weights[4].*TO.stage_cost(const_fn, x, controls[i])))
                write(file, ", ")
            end
        end    
    end


    touch("jointvel.txt")
    open("jointvel.txt", "w") do file
        for i =1:length(trajectory)
            const_fn = jointvel_cost_fns[i]
            x = trajectory[i]
    
            if const_fn.is_terminal
                write(file, string(weights[5].*TO.stage_cost(const_fn, x)))
                write(file, "\n")
            else
                write(file, string(weights[5].*TO.stage_cost(const_fn, x, controls[i])))
                write(file, ", ")
            end
        end    
    end

    filer = string("traj_", file_ending, ".txt");
    touch(filer)
    open(filer, "w") do file
        for angular in trajectory
            write(file, string(angular)[2:end-1])
            write(file, "\n")
        end
        write(file, "0.25, 0.0")
    end
    println("Done Printing Trajectory")
    run(`python plot_data.py $file_ending`)
    println("Executing CoMOTO Trajectory")
    confirm_display_traj(TO.states(solver), "JACO", dt*(n_timesteps - 1), means_path)
    # println("Executing Nominal Trajectory")
    # confirm_display_traj([SVector{7}(prob_info.nominal_traj[:,t]) for t=1:n_timesteps], "JACO", dt*(n_timesteps - 1), "human_traj_data/means_103.csv")

end

main()
