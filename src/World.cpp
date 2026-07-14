#include "rbd/World.hpp"
#include "rbd/Integrator.hpp"

namespace rbd {

World::World() : gravity(0.0, -9.81, 0.0) {}

void World::addBody(RigidBody* body) {
    bodies.push_back(body);
}

void World::step(double dt) {
    for (auto* body : bodies) {
        // Apply gravity force (F = m * g)
        body->applyForce(gravity * body->mass);
        
        // Integrate state
        RK4Integrator::step(*body, dt);
        
        // Clear accumulators for the next step
        body->clearAccumulators();
    }
}

} // namespace rbd