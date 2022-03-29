## Summary
This paper uses an SMT solver (which alternates a SAT solver with a numerical solver) to continually re-solve controllers for a differential drive robot that meet certain constraints with regard to robot navigation (non-collision, move next to human). This is nice because:
- You get to say how you want movement to look instead of specifying some policy or objective function
- If a problem is infeasible, the solver can tell you why

## Relevant Approach
The cool things here are:

1) Top-down approach
2) Solving a new controller every timestep
3) You get 1 and 2 without losing strong constraints

## Limitations
This work doesn't solve an entire trajectory at once or consider predictions beyond the next timestep. This means that you can hit local minima pretty easily. That's OK for their use case (humans are responsible for constraint violation), but maybe not for ours.

## Relevant Citations