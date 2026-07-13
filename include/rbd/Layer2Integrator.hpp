#pragma once
#include <Eigen/Dense>

namespace rbd {

class Layer2Integrator {
public:
    // Generic RK4 solver specifically for state-space formulations (X_dot = f(X, t))
    template <typename System>
    static void step(System& sys, double dt) {
        Eigen::Vector2d zero_tau(0.0, 0.0); // Zero external torque for energy validation

        Eigen::Vector4d k1 = sys.computeDerivatives(sys.state, zero_tau);
        Eigen::Vector4d k2 = sys.computeDerivatives(sys.state + 0.5 * dt * k1, zero_tau);
        Eigen::Vector4d k3 = sys.computeDerivatives(sys.state + 0.5 * dt * k2, zero_tau);
        Eigen::Vector4d k4 = sys.computeDerivatives(sys.state + dt * k3, zero_tau);

        sys.state += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
};

} // namespace rbd