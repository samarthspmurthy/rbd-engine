# C++ Rigid Body Dynamics Engine

## Project Overview
This project is a custom-built, fixed-step Rigid Body Dynamics (RBD) engine written in C++, validated against commercial variable-step solvers in MATLAB/Simscape. As a mechanical engineer, I built this engine to move beyond simply using simulation software to actively engineering the mathematics and software architecture that power it. 

This repository demonstrates a complete, end-to-end physics simulation pipeline: deriving the kinematics and dynamics (KDMS), building a robust Object-Oriented C++ solver, and establishing a rigorous data-validation loop in MATLAB. It serves as a capstone demonstration of my technical capabilities for the MathWorks Engineering Development Group (EDG) program, bridging the gap between mechanical theory, control systems, and high-performance software engineering.

## Mathematical Foundation
The engine avoids "black box" physics libraries, relying entirely on hand-derived mathematics implemented via the Eigen linear algebra library. 
* **Free-Body Dynamics:** Governed by the Newton-Euler equations.
* **Multi-Link Dynamics:** The Two-Link Arm is modeled using Lagrangian mechanics, calculating the Mass Matrix, Coriolis matrix, and Gravity vector ($M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$).
* **Numerical Integration:** Employs a custom Runge-Kutta 4 (RK4) integrator for high-fidelity state advancement.

*(Note: All mathematical derivations, including inertia tensor transformations and relative-angle forward kinematics, were hand-derived prior to implementation to ensure architectural soundness).*

## Architecture Overview
The C++ source is modularized for clarity and extensibility:
* `RigidBody` / `InertiaTensor`: Base classes handling mass properties and state vectors.
* `Quaternion`: Custom kinematics class ensuring singularity-free rotation tracking.
* `TwoLinkArm`: The core Lagrangian dynamics solver for the double pendulum system.
* `World` / `Integrator`: The simulation environment and RK4 time-stepping logic.
* `DataLogger`: CSV telemetry export for offline MATLAB validation.

## Key Engineering Decisions & Tradeoffs
* **Quaternions vs. Euler Angles:** Chose quaternions for the core rigid body state to completely eliminate gimbal lock during complex 3D rotations, despite the added mathematical overhead.
* **Body-Frame vs. World-Frame Inertia:** Inertia tensors are defined strictly in the body frame (where they remain constant) and transformed dynamically via rotation matrices, avoiding expensive recalculations of the world-frame tensor at every time step.
* **Reduced Coordinates vs. Generalized Constraints:** For the multi-link arm, I deliberately scoped the engine to use reduced-coordinate Lagrangian dynamics rather than a generalized Lagrange-multiplier constraint solver. This was a conscious decision to prioritize absolute numerical stability and energy conservation in a chaotic system before generalizing the joint architecture.

## Validation Methodology & Results
The C++ engine's outputs were validated against a structurally identical Simscape Multibody model. Given that a double pendulum is a chaotic system (Lyapunov instability), long-term numerical drift is inevitable between a fixed-step RK4 solver ($dt = 0.001$) and a variable-step commercial solver (`ode4`). 

Despite this chaotic nature, the engine demonstrated exceptional accuracy:
* **Joint Angle Max Error:** $< 1.0 \times 10^{-5}$ rad over a full 5-second simulation.
* **Forward Kinematics (End-Effector):** Near-perfect Cartesian overlay, confirming the structural integrity of the transformation matrices.
* **Energy Conservation:** Remained fully stable without artificial damping.

Read the full validation methodology and error analysis here: [Validation Report](validation_report.md)

## Build Instructions
This project requires CMake and a C++17 compatible compiler. The `Eigen` library is included as a header-only dependency.
```bash
mkdir build
cd build
cmake ..
make
./rbd_engine