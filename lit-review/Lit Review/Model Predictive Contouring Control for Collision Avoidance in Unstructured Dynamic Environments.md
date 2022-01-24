## Summary
The paper proposes a variation on MPC called "Local Model Predictive Contouring Control" for doing path planning and control based on model predictions

## Relevant Approach
A number of good ideas here. Ellipsoidal collision sets. They also have a model where a nominal trajectory is generated based on human-agnostic costs, then deviations from that trajectory are calculated online using the minimum number of segments. You get goal-reaching from the tracking cost of the nominal trajectory (could be an LQR).

Actual problem solved as a receding horizon nonconvex optimization problem. They find sub-50ms solve times on a pretty slow CPU.

## Limitations
As usual, collision-only - but no reason the general framework couldn't be extended to CoMOTO

## Relevant Citations