## Brainstorm
Probabalistic prediction
Theoretical guarantees
Reactive & anticipatory MP


- Very rough draft of a paper
- Intro
- Related Works
- Problem formulation
	- Mathematical model
	- Identify available information/control types
- Approach

## Intro and Related Works
In tasks where humans and robots must work together in shared workspaces, the robots must be able to model the activities of team members and adjust their own activities accordingly, to make themselves more legible and . This holds true in a task space, as in [[Ten Challenges for Making Automation a “Team Player” in Joint Human-Agent Activity|10 Challenges]], but also in the motion planning space, where robots need to plan safe and comfortable trajectories in order to avoid placing a cognitive burden on their human partners ((citation needed))

    Note: I haven't found significant top-down work on measuring the impact of comfortable motion or even defining it. This seems odd.

A great deal of research has gone into the planning of safe trajectories, which can be defined as trajectories which do not cause the robot to collide with objects in its environment. [[Bridging the gap between safety and real-time performance in receding-horizon trajectory design for mobile robots|Bridging the Gap...]] gives a useful overview of these methods, which can roughly be broken into sample-based (such as RRT), Nonlinear Model Predictive Control (NMPC, which generally formulates the problem as a constrained system of nonlinear equations and solves it), and reachability-based, like [[Funnel libraries for real-time robust feedback motion planning|Funnel Libraries]]

However, less work has gone into defining or generating comfortable motion. [[Legibility]] focuses on a subset of comfortable motion, legibility, which is defined as a property of motion which makes it easy for an observer to predict its goal or endpoint. 

    Note: need more sources on generating comfortable motion, if possible

In our previous work, [[CoMOTO_v1]], we presented CoMOTO, an approach to motion planning in shared workspaces via NMPC. In this work, we extend CoMOTO to a real-time setting by performing receding-horizon planning. Through this approach, the robot is able to replan **with more conservative safety parameters** when the model fails to correctly predict human motion. In this work, we focus on generating trajectories for a 7 degrees of freedom robotic manipulator, but this method could be applied with minimal modification to other types of robots.

## Problem Formulation
Given a predicted human trajectory $H_t$, we wish to compute an optimal discrete-time trajectory $x_t, u_t$. In our work, we employ joint-space velocity control; therefore, $x_t$ will be a vector of joint angles and $u_t$ a vector of joint-space angular velocities. We employ an NMPC approach, formulating the following problem:

$$ minimize\ C(x_t, u_t) $$
$$ s.t.\ \ L^{(i)} < x_t^{(i)} < U^{(i)} $$

Where $L$ and $U$ are vectors representing the lower and upper joint limits for the manipulator. As this is a receding-horizon problem, a second constraint might be added in some runs to ensure the robot reaches its goal state. The cost function $C$ is discussed in the Approach section.

## Approach
In our prior work, we formulated an overall cost function as a weighted sum of five objectives. In this work, we reformulate several of these objectives to allow each timestep in the trajectory to be optimized independently of the others.

### Distance

### Visibility

### Legibility
(Note: not edited for readability)

Legibility is defined by Dragan et. al. as

$$\dfrac{\int P(G_R | \xi_{S \rightarrow \xi(t)}) f(t) dt}{\int f(t)} $$

The probability term in the cost above is found to be

$$ P(G_R | \xi_{S \rightarrow \xi(t)}) = \dfrac{1}{Z} \dfrac{exp(-C[\xi_{S \rightarrow Q}] - V_{G_R}(Q)}{exp(-V_{G_R}(S))} $$

Where $Z$ is a normalizer ($Z = \sum_G  \dfrac{exp(-C[\xi_{S \rightarrow Q}] - V_{G}(Q)}{exp(-V_{G}(S))}$), $C$ is a cost functional, defined for our code as the square of the path length, and $V_A(X)$ is defined as the cost of the lowest-cost path from $X$ to $A$ ($\texttt{min}_\xi C[\xi] | \xi(0) = X, \xi(end) = A$). 

The formulas above are implemented in trajopt almost as they appears here, using a summation to approximate the integral and piecewise distances between timesteps to approximate path lengths for $C$. To account for multiple candidate goal locations, we consider the legibility cost at a timestep $t$ to be:

$$ \textsc{LegibilityCost}[\xi, t] = 1 - P(G^* | \xi_{S \rightarrow \xi(t)})  $$

The trajopt formulation couples the trajectory waypoints, making optimization more difficult. In Julia, we decouple the waypoints by modifying probability function a bit:

$$ P(G_R | \xi_{S \rightarrow \xi(t)}) = \dfrac{1}{Z} \dfrac{exp(-||Q - S||^2 -||G_R - Q||^2}{exp(-||G_R - S||^2)} $$

This first incorporates our definition of $C$ as the square path length, then assumes that the path from the start of the trajectory to a given waypoint has been cartesian-linear. Note, however, that this $||Q - S||^2$ term can now be removed from the calculation altogether for the purposes of calculating legibility cost. This is because it isn't parameterized on the goal location, so it will be removed from the calculation of $P(G^* | \xi_{S \rightarrow \xi(t)})$ by the normalizer $Z$. The actual calculation performed in Julia, then, is:

$$ \textsc{LegibilityCost}[\xi, t] = 1 - \dfrac{1}{Z} \texttt{exp}(||G^* - \xi(0)||^2 - ||G^* - \xi(t)||^2)  $$

### Nominal

### Smoothness


## Experimental Setup