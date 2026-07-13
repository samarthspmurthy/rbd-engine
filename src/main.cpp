#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "rbd/RigidBody.hpp"
#include "rbd/World.hpp"

int main() {
    std::cout << "--- RBD Engine: Phase 3 (Tennis Racket Instability) ---" << std::endl;

    // 1. Create a Zero-Gravity World
    rbd::World world;
    world.gravity.setZero(); 

    // 2. Initialize the Asymmetric Rigid Body
    rbd::RigidBody body;
    body.setMass(1.0); 

    // 3. Define the Asymmetric Inertia Tensor (Three completely unique axes)
    // I_xx = 0.1 (Smallest), I_yy = 0.4 (Intermediate), I_zz = 0.5 (Largest)
    Eigen::Matrix3d asymmetric_inertia;
    asymmetric_inertia << 0.1, 0.0, 0.0,
                          0.0, 0.4, 0.0,
                          0.0, 0.0, 0.5;
    body.setInertiaTensor(asymmetric_inertia);

    // 4. Spin concentrated on the Intermediate Axis (Y)
    // A tiny perturbation (0.01) on X kicks off the physical instability
    body.linear_velocity.setZero();
    body.angular_velocity = Eigen::Vector3d(0.01, 5.0, 0.0);
    
    world.addBody(&body);

    // 5. Simulation Loop Parameters
    double dt = 0.01;      // 10 millisecond steps
    double end_time = 2.5; // 2.5 seconds gives enough time to watch the flip occur
    int steps = static_cast<int>(end_time / dt);

    std::cout << "\nSimulating free-body rotation on the Intermediate Axis..." << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\nTime(s)  |  AngVel_X  |  AngVel_Y  |  AngVel_Z" << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;

    // 6. Run the Simulation Loop
    for (int i = 0; i <= steps; ++i) {
        // Print telemetry every 20 steps to watch the trajectory trend
        if (i % 20 == 0) {
            std::cout << std::setw(6) << (i * dt) << "s | " 
                      << std::setw(10) << body.angular_velocity.x() << " | "
                      << std::setw(10) << body.angular_velocity.y() << " | "
                      << std::setw(10) << body.angular_velocity.z() << std::endl;
        }

        // Integrate time step inside the world
        if (i < steps) {
            world.step(dt);
        }
    }

    return 0;
}