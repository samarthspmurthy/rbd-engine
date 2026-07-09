#include <iostream>
#include <Eigen/Dense>
#include "rbd/RigidBody.hpp"
#include "rbd/Integrator.hpp"

int main() {
    std::cout << "--- RBD Engine: Phase 1 Simulation ---" << std::endl;

    // 1. Initialize the Rigid Body
    rbd::RigidBody body;

    // 2. Set the initial state
    body.linear_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
    body.angular_velocity = Eigen::Vector3d(0.0, 2.0, 0.0);

    std::cout << "\n[T = 0.0s] Initial State:" << std::endl;
    std::cout << "Position:   [" << body.position.transpose() << "]" << std::endl;
    std::cout << "Quaternion: [" << body.orientation.transpose() << "]" << std::endl;

    // 3. Simulation Loop Parameters
    double dt = 0.01;      // Time step of 10 milliseconds
    double end_time = 1.0; // Simulate for exactly 1 second
    int steps = static_cast<int>(end_time / dt); // Explicit cast to fix the C4244 warning

    // 4. Run the RK4 Integrator
    std::cout << "\nSimulating for 1.0 second at dt = 0.01s..." << std::endl;
    for (int i = 0; i < steps; ++i) {
        rbd::RK4Integrator::step(body, dt);
    }

    // 5. Print the Final State
    std::cout << "\n[T = 1.0s] Final State:" << std::endl;
    std::cout << "Position:   [" << body.position.transpose() << "]  <-- Should be [1, 0, 0]" << std::endl;
    std::cout << "Quaternion: [" << body.orientation.transpose() << "]" << std::endl;
    std::cout << "Lin Vel:    [" << body.linear_velocity.transpose() << "]  <-- Should be constant" << std::endl;
    std::cout << "Ang Vel:    [" << body.angular_velocity.transpose() << "]  <-- Should be constant" << std::endl;

    return 0;
}