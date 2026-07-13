#include <iostream>
#include <iomanip>
#include "rbd/DoublePendulum.hpp"
#include "rbd/Layer2Integrator.hpp" 

int main() {
    std::cout << "--- RBD Engine: Phase 4 (Double Pendulum Energy Validation) ---" << std::endl;

    rbd::DoublePendulum pendulum;

    // Initial state: Drop from horizontal (theta1 = 0 rad, theta2 = 0 rad)
    // Both angular velocities start at 0
    pendulum.state << 0.0, 0.0, 0.0, 0.0;

    double dt = 0.001; // 1 ms timestep for high-precision integration
    double end_time = 5.0; // Simulate 5 seconds of chaotic swinging
    int steps = static_cast<int>(end_time / dt);

    std::cout << "\nSimulating conservative chaotic swing (zero torque, zero damping)..." << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nTime(s) |  Theta 1  |  Theta 2  |  Total Energy (J)" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;

    double initial_energy = pendulum.getTotalEnergy();

    for (int i = 0; i <= steps; ++i) {
        if (i % 500 == 0) { // Print every 0.5 seconds
            std::cout << std::setw(6) << (i * dt) << "s | " 
                      << std::setw(9) << pendulum.state(0) << " | "
                      << std::setw(9) << pendulum.state(1) << " | "
                      << std::setw(14) << pendulum.getTotalEnergy() << std::endl;
        }

        if (i < steps) {
            rbd::Layer2Integrator::step(pendulum, dt); 
        }
    }

    std::cout << "\n[Validation] Initial Energy: " << initial_energy << " J" << std::endl;
    std::cout << "[Validation] Final Energy:   " << pendulum.getTotalEnergy() << " J" << std::endl;

    return 0;
}