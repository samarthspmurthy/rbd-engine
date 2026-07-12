#include <iostream>
#include <Eigen/Dense>
#include "rbd/RigidBody.hpp"
#include "rbd/InertiaTensor.hpp"
#include "rbd/World.hpp"

int main() {
    std::cout << "--- RBD Engine: Phase 2 (World Gravity & Projectile Motion) ---" << std::endl;

    // 1. Create the environment
    rbd::World world;

    // 2. Create the body and set physical properties using the new primitives
    rbd::RigidBody box;
    box.setMass(2.0); // 2.0 kg
    
    // Create a 1m x 2m x 1m box
    Eigen::Matrix3d box_inertia = rbd::InertiaTensor::createSolidBox(box.mass, 1.0, 2.0, 1.0);
    box.setInertiaTensor(box_inertia);

    // 3. Set Initial State (Projectile motion: 5 m/s forward on X)
    box.linear_velocity = Eigen::Vector3d(5.0, 0.0, 0.0);
    
    // Add box to the world
    world.addBody(&box);

    std::cout << "\n[T = 0.0s] Initial State:" << std::endl;
    std::cout << "Position: [" << box.position.transpose() << "]" << std::endl;
    std::cout << "Lin Vel:  [" << box.linear_velocity.transpose() << "]" << std::endl;

    // 4. Simulation Loop (1 second)
    double dt = 0.01;
    double end_time = 1.0;
    int steps = static_cast<int>(end_time / dt);

    std::cout << "\nSimulating projectile motion in World with gravity..." << std::endl;
    for (int i = 0; i < steps; ++i) {
        world.step(dt); // Gravity and RK4 integration handled internally by World
    }

    // 5. Print Output
    std::cout << "\n[T = 1.0s] Final State:" << std::endl;
    std::cout << "Position: [" << box.position.transpose() << "]  <-- Should be [5, -4.905, 0]" << std::endl;
    std::cout << "Lin Vel:  [" << box.linear_velocity.transpose() << "]  <-- Should be [5, -9.81, 0]" << std::endl;
    std::cout << "Ang Vel:  [" << box.angular_velocity.transpose() << "]  <-- Should be [0, 0, 0] (no rotation)" << std::endl;

    return 0;
}