# CoMOTO.jl 
## CoMOTO implementation in Julia

### Installation
Install Julia for your OS. Download this repository and cd into it, then open Julia (so that `@__DIR__` is the directory containing this code). Run these commands:

```
julia> import Pkg; Pkg.activate(@__DIR__);
julia> Pkg.instantiate();
```

This should install all packages required to run this code in a Julia environment specific to this project. The current code (currently in early beta mode) can be run from the terminal with:

```
$ cd src
$ julia comoto.jl
```

#### ROS
`RobotOS.jl` makes Python calls and maintains its own Python environment, meaning that you need to preinstall `rospy` in the Julia environment. 

```
julia> using Conda; Conda.add("ros-rospy", channel="conda-forge");
```

Rviz offset: the RViz model used in the RAIL pipeline is up on a table, which offsets it from the model used in this code, which is not on a table. The offset is, approximately:

```
jl_pos = rviz_pos + [0.5, 0, -0.75]
```

### Files
Aside from `main.jl`, there are a few other files in this project:

- Funnels.ipynb: a Jupyter notebook that was used for prototyping early iterations of this code. It is not maintained and should not do anything.
- kuka.urdf: An urdf file containing information for a kuka iiwa 14. This enables the use of forward kinematics in calculating costs.
- Manifest.toml and Project.toml: standard Julia package files
- means.csv: A human trajectory, taken from the original CoMOTO implementation and annotated
- vars.csv: A human variance trajectory associated with means.csv, taken from the original CoMOTO implementation