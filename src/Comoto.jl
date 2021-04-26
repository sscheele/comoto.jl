module Comoto

using RobotDynamics, Rotations
using TrajectoryOptimization
using StaticArrays, LinearAlgebra
using RigidBodyDynamics;
using Altro;
import ForwardDiff;
using Dates;
import CSV;
import Dierckx;

const OBJECT_SET = SArray{Tuple{3,2}}(reshape([0.752,-0.19,0.089, 0.752, 0.09, -0.089], (3,2)));
const OBJECT_POS = OBJECT_SET[:,1];

const RBD = RigidBodyDynamics;
const TO = TrajectoryOptimization;

include("ros_interface.jl")
include("util.jl")
include("problem_info.jl")
include("main.jl")

end # module