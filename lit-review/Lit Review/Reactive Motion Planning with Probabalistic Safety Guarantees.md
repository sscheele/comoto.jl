Paper title: Reactive Motion Planning with Probabalistic Safety Guarantees
Paper year: 

## Summary
- Proposes a learning framework that predicts motion over a short time horizon
- Framework is demonstrated to be probabalistically correct
- Shows that [[MPC]] (posed as an optimization problem) is able to use the predictive framework to perform highway driving


## Approach
- The framework outputs a set of possible trajectories, not a single prediction
- The framework uses a trajectory basis set and works as a classifier - it predicts which trajectories in its basis set the object might follow (each one recieves a 0/1 classification)
- For the MPC, each waypoint in the trajectory is associated with an ellipsoid, and an optimization problem is solved such that that ellipsoids do not intersect (a slack variable is added such that collisions are possible, but penalized)
	- MPC is solved is CasADi autodiff and IPOPT

## Limitations
- Only non-collision constraint is considered

## Relevant Citations

Bemporad et al Predictive control for linear and hybrid systems
- MPC book
