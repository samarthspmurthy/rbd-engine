#pragma once
#include <vector>
#include <Eigen/Dense>
#include "RigidBody.hpp"
#include "Integrator.hpp"

namespace rbd {

class World {
public:
    std::vector<RigidBody*> bodies;
    Eigen::Vector3d gravity;

    // Default gravity vector (-9.81 m/s^2 on the Y axis)
    World() : gravity(0.0, -9.81, 0.0) {}

    void addBody(RigidBody* body) {
        bodies.push_back(body);
    }

    void step(double dt) {
        for (auto* body : bodies) {
            // Apply gravity force (F = m * g)
            body->applyForce(gravity * body->mass);
            
            // Integrate state
            RK4Integrator::step(*body, dt);
            
            // Clear accumulators for the next step
            body->clearAccumulators();
        }
    }
};

} // namespace rbd