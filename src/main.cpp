#include <iostream>
#include <Eigen/Dense>
#include "rbd/RigidBody.hpp"
#include "rbd/Integrator.hpp"

int main() {
    std::cout << "--- RBD Engine: Phase 2 Dynamics (Gravity & Torque) ---" << std::endl;

    // 1. Initialize the Rigid Body
    rbd::RigidBody body;

    // 2. Set Mass and Inertia (Solid Sphere, r=1.0m)
    body.setMass(2.0); // 2.0 kg
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity() * 0.8; // 0.8 kg*m^2
    body.setInertiaTensor(inertia);

    std::cout << "\n[T = 0.0s] Initial State:" << std::endl;
    std::cout << "Position:   [" << body.position.transpose() << "]" << std::endl;
    std::cout << "Lin Vel:    [" << body.linear_velocity.transpose() << "]" << std::endl;
    std::cout << "Ang Vel:    [" << body.angular_velocity.transpose() << "]" << std::endl;

    // 3. Simulation Loop Parameters
    double dt = 0.01;      // Time step of 10 milliseconds
    double end_time = 1.0; // Simulate for exactly 1 second
    int steps = static_cast<int>(end_time / dt);

    std::cout << "\nApplying Gravity (-9.81 in Y) and Constant Torque (0.5 in Z)..." << std::endl;

    // 4. Run the Physics Loop
    for (int i = 0; i < steps; ++i) {
        // Apply Force: Weight = mass * gravity
        Eigen::Vector3d gravity(0.0, -9.81 * body.mass, 0.0);
        body.applyForce(gravity);
        
        // Apply Torque
        Eigen::Vector3d torque(0.0, 0.0, 0.5);
        body.applyTorque(torque);

        // Integrate one time step
        rbd::RK4Integrator::step(body, dt);

        // CRITICAL: Clear forces/torques after they have been processed
        // so they don't exponentially stack up on the next loop iteration!
        body.clearAccumulators();
    }

    // 5. Print the Final State
    std::cout << "\n[T = 1.0s] Final State:" << std::endl;
    std::cout << "Position:   [" << body.position.transpose() << "]  <-- Should be [0, -4.905, 0]" << std::endl;
    std::cout << "Lin Vel:    [" << body.linear_velocity.transpose() << "]  <-- Should be [0, -9.81, 0]" << std::endl;
    std::cout << "Ang Vel:    [" << body.angular_velocity.transpose() << "]  <-- Should be [0, 0, 0.625]" << std::endl;

    return 0;
}