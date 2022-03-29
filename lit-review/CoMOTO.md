## Ideas
- Optimize in Cartesian space, then another round of optimization for ik
	+ Goal can now be specified as a cartesian pose
	+ Maybe faster?
		+ Test RRT trellis
	- No guarantees that a good cartesian trajectory can be realized (singularities, joint limits, etc)
		- Maybe add a "near joint limit" cost to cartesian optimization?
- Reuse prior solutions
	- Update: this is difficult because the same controls can't be used as the initialization next time
- High-level planning + low-level optimization
- Combination idea:
	1. Have goal as a cartesian pose
	2. Sample an ik solution for goal pose for initialization only
	3. Add a cost function (and maybe constraint) that drives robot to finish at goal pose
	- This should allow the robot to implicitly optimize the ik solution and maybe prevent awkward solutions
- Null out gradients below a certain epsilon (might allow faster optimization)

## Basics of the Field
A 3-level hierarchical architecture is normally used:
- Graph search to find some coarse route that avoids static or known-dynamic obstacles (PRM or RRT)
- Trajectory planning to optimize the graph search results (and, if graph search was done in cartesian-space, transform it into joint space)
- Tracking control to realize the route as accurately as possible

For trajectory planning, there are 3 basic approaches:
- Sample-based (eg: RRTs or [[FMT]]s): most intuitive with static obstacles. Different possible trajectories are computed by sampling the goal or state space and forward-integrating the robot's dynamics appropriately. Then a path through the graph is computed. 
	- Question: how would this work with dynamic obstacles?
- [[Nonlinear Model Predictive Control]] - the robot and environment are assumed to follow certain dynamics, then optimal parameters are computed for discrete timesteps via nonlinear programming
	- #todo: look into Sequential Action Control
- Reachability-based methods - this is most closely what Tedrake and Majumdar were doing with their funnel libraries. The reachable set of the robot is computed for various control(ler)s (sometimes offline), then a control is selected such that the reachable set doesn't intersect with any obstacles.

## Our Approach
**Warrant**:
- Prior work has demonstrated that optimizing for subjective human comfort improves human and overall performance in shared workspaces
- However, unclear that prior work has optimized well (or what optimizing well means, from a bottom-up perspective)

**Prior work**:
- Dragan optimized for legibility

**Novelty**:
- We simultaneously optimize for a number of functions
- We use a receding horizon model with re-solves in real time, so we can do reactive MP

## Basis
Probabalistic prediction
Theoretical guarantees
Reactive & anticipatory MP


## todo 
- [ ] Visualization:
	- [ ] Report statistics vs full re-solve
- [ ] Reorganize code (need Hritik code)
- [ ] Get thesis permit (title: ) #todo 
- [ ] Lit Review
	- [ ] Anca Dragan
	- [ ] Dorsah Sadigh
	- [ ] Sidd Srinivasa
	- [ ] Marc Toussaint
	- [ ] Leslie Kaelbling
	- [ ] Henny Admonia
	- [ ] David Held
	- [ ] Changlin Liu