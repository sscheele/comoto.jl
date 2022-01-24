## Summary
SHARP is simply a framework for planning in a way that respects "shielding" (replacement of the nominal plan with a fallback strategy focused on safety). 

## Relevant Approach
SHARP formulates a problem as a stochastic optimal control problem to avoid unnecesarily activating the shielding strategy (and sometimes preempting it). Per the framework, this could theoretically be an arbitrary cost function. However, much of the work assumes that the cost function is some linear or quadratic cost, so that the whole problem can be posed as a quadratic program.

## Limitations
Best for linear/quadratic costs. Assumes Bayesian inference for human motion

## Relevant Citations

[[Probabilistically Safe Robot Planning with Confidence-Based Human Predictions]]
