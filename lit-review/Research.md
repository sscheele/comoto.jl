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
	

## Basis
Probabalistic prediction
Theoretical guarantees
Reactive & anticipatory MP


## todo 
- [ ] Visualization:
	- [ ] Report statistics vs full re-solve
- [x] Help Hritik (Tue 09/14) ✅ 2021-09-16
- [ ] Reorganize code (need Hritik code)
- [ ] Lit Review
	- [ ] Anca Dragan
	- [ ] Dorsah Sadigh
	- [ ] Sidd Srinivasa
	- [ ] Marc Toussaint
	- [ ] Leslie Kaelbling
	- [ ] Henny Admonia
	- [ ] David Held
	- [ ] Changlin Liu