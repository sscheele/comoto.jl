include("comoto_jaco.jl")

function main(human_trajfile::String="human_traj_data/means.csv")
    @sync begin
        human_dt = total_time/(countlines(human_trajfile)-1);
        if human_trajfile != ""
            @async dispatch_human_trajectory(human_trajfile, human_dt);
        end
    end
end