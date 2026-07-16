#include <iostream>

#include <iomanip>

#include <cmath>

#include <vector>

#include <string>

#include "rbd/DoublePendulum.hpp"

#include "rbd/Layer2Integrator.hpp" 

#include "rbd/DataLogger.hpp" // NEW: Include the logger



int main() {

    std::cout << "--- RBD Engine: Phase 5 (Data Logging) ---" << std::endl;



    rbd::DoublePendulum pendulum;

    pendulum.state << 0.0, 0.0, 0.0, 0.0;



    double dt = 0.001; 

    double end_time = 5.0; 

    int steps = static_cast<int>(end_time / dt);



    // 1. Initialize the Logger

    // NOTE: Make sure the "data/output/" directory actually exists on your computer!

    rbd::DataLogger logger("data/output/simulation_log.csv");



    // 2. Write the CSV Header

    std::vector<std::string> headers = {

        "theta1", "theta2", "theta1_dot", "theta2_dot",

        "elbow_x", "elbow_y", "ee_x", "ee_y", "total_energy"

    };

    logger.writeHeader(headers);



    std::cout << "\nSimulating conservative chaotic swing and logging to CSV..." << std::endl;

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "\nTime(s) |  Theta 1  |  Theta 2  |  Total Energy (J)" << std::endl;

    std::cout << "-----------------------------------------------------" << std::endl;



    double initial_energy = pendulum.getTotalEnergy();



    for (int i = 0; i <= steps; ++i) {

        double current_time = i * dt;

        

        // Extract current state for clean math

        double t1 = pendulum.state(0);

        double t2 = pendulum.state(1);

        double t1_dot = pendulum.state(2);

        double t2_dot = pendulum.state(3);



        // 3. Forward Kinematics (KDMS)

        double elbow_x = pendulum.l1 * cos(t1);

        double elbow_y = pendulum.l1 * sin(t1);

        double ee_x = elbow_x + pendulum.l2 * cos(t1 + t2);

        double ee_y = elbow_y + pendulum.l2 * sin(t1 + t2);

        double energy = pendulum.getTotalEnergy();



        // 4. Log the row to CSV (every single timestep)

        logger.logRow(current_time, {t1, t2, t1_dot, t2_dot, elbow_x, elbow_y, ee_x, ee_y, energy});



        // Console output (every 0.5 seconds / 500 steps)

        if (i % 500 == 0) { 

            std::cout << std::setw(6) << current_time << "s | " 

                      << std::setw(9) << t1 << " | "

                      << std::setw(9) << t2 << " | "

                      << std::setw(14) << energy << std::endl;

        }



        // Integrate forward

        if (i < steps) {

            rbd::Layer2Integrator::step(pendulum, dt); 

        }

    }



    std::cout << "\n[Validation] Initial Energy: " << initial_energy << " J" << std::endl;

    std::cout << "[Validation] Final Energy:   " << pendulum.getTotalEnergy() << " J" << std::endl;

    std::cout << "[Success] Data successfully written to data/output/simulation_log.csv" << std::endl;



    return 0;

} 