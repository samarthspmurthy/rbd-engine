#pragma once
#include <Eigen/Dense>
#include "RigidBody.hpp"


namespace rbd {

class RK4Integrator {
public:
    // Steps the rigid body physics forward by 'dt' seconds
    static void step(RigidBody& body, double dt) {
        // Grab the starting state
        Eigen::VectorXd initial_state = body.getState();
        
        // k1: Derivative at the beginning of the interval
        Eigen::VectorXd k1 = body.computeStateDerivative();
        
        // k2: Derivative at the midpoint (using k1)
        body.setState(initial_state + 0.5 * dt * k1);
        Eigen::VectorXd k2 = body.computeStateDerivative();
        
        // k3: Derivative at the midpoint (using k2)
        body.setState(initial_state + 0.5 * dt * k2);
        Eigen::VectorXd k3 = body.computeStateDerivative();
        
        // k4: Derivative at the end of the interval (using k3)
        body.setState(initial_state + dt * k3);
        Eigen::VectorXd k4 = body.computeStateDerivative();
        
        // Final State Update using the weighted average of the slopes
        Eigen::VectorXd final_state = initial_state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        
        // Apply the newly integrated state back to the body
        body.setState(final_state);
    }
};

} // namespace rbd
// Layer 2: Generic Reduced-Coordinate Integrator (RK4 doesn't care what the system is!)
    template <typename System>
    static void step(System& sys, double dt) {
        Eigen::Vector2d zero_tau(0.0, 0.0); // Zero external torque for validation

        Eigen::Vector4d k1 = sys.computeDerivatives(sys.state, zero_tau);
        Eigen::Vector4d k2 = sys.computeDerivatives(sys.state + 0.5 * dt * k1, zero_tau);
        Eigen::Vector4d k3 = sys.computeDerivatives(sys.state + 0.5 * dt * k2, zero_tau);
        Eigen::Vector4d k4 = sys.computeDerivatives(sys.state + dt * k3, zero_tau);

        sys.state += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }