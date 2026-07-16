# Phase 7: Rigid Body Dynamics Validation

## Objective
To validate the accuracy of a custom C++ physics engine (Fixed-step RK4) against a commercial baseline (MATLAB Simscape Multibody `ode4`) by simulating a chaotic double pendulum system for 5 seconds.

## Methodology
1. **Engine Setup:** Developed a C++ Two-Link Arm simulation utilizing Lagrangian mechanics, a custom math vector library, and an RK4 integrator ($dt = 0.001$ s).
2. **Baseline Model:** Built an equivalent Simscape Multibody model using standard SI parameters ($m = 1.0$ kg, $l = 1.0$ m) and the `ode4` solver.
3. **Data Pipeline:** Exported time-series data from both environments and developed a MATLAB script to compute absolute errors and validate Forward Kinematics.

## Results
* **Core Dynamics:** Joint angles ($\theta_1$, $\theta_2$) and velocities matched the commercial solver with an absolute maximum error bounded strictly under $10^{-5}$ radians over 5 seconds.
* **Forward Kinematics:** Cartesian coordinates for the elbow and end-effector showed near-perfect overlap, proving the relative-angle transformation matrices are structurally sound.
* **Energy Conservation:** Total system energy remained stable, confirming the absence of artificial energy injection from the RK4 solver.

## Engineering Discussion
Given the highly chaotic nature of the double pendulum (Lyapunov exponent $> 0$), long-term divergence between discrete solvers is inevitable due to floating-point truncation and algorithmic differences. Maintaining a sub-$10^{-5}$ radian error threshold over 5 seconds of highly non-linear dynamics confirms that the underlying Equations of Motion (Mass Matrix, Coriolis, and Gravity vectors) are highly accurate and numerically stable.