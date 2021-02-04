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
$ julia main.jl
```