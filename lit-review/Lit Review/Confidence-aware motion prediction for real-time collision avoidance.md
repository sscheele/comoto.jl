## Summary
The primary contribution of the paper is that it analyzes model performance in real-time to adjust obstacle margins. 

## Relevant Approach
It maintains several models and Bayesian priors representing confidence in them, and extends FasTrack to avoid the funnel formed by the occupancy probability distribution.

## Limitations
Collision-only, doesn't seem to be convincingly better than funnel-based approaches (which will be faster)

## Relevant Citations