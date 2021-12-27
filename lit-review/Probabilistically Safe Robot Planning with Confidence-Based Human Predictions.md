## Summary
The paper constructs a Bayesian prediction of a trajectory and updates it as the person moves to produce confidence intervals, which are then used to help a drone avoid collisions.

The work uses a "rationality coefficient" to measure their model's performance.

## Relevant Approach
The paper uses IRL to predict human action and keeps track of predicted behaviors and states by recalculating a "model confidence", modeled as a Bayesian prior, after every measurement update. It then uses traditional planning software to avoid collisions.

## Limitations
Collision-only

## Relevant Citations
Guided Cost Learning: Deep Inverse Optimal Control via Policy Optimization