#pragma once
#include <Eigen/Dense>
#include "Quaternion.hpp"

namespace rbd {

class RigidBody {
public:
    // 13-Dimensional State
    Eigen::Vector3d position;          // r: [x, y, z]
    Eigen::Vector4d orientation;       // q: [qw, qx, qy, qz]
    Eigen::Vector3d linear_velocity;   // v: [vx, vy, vz]
    Eigen::Vector3d angular_velocity;  // w: [wx, wy, wz]

    RigidBody() {
        position.setZero();
        orientation << 1.0, 0.0, 0.0, 0.0; // Identity quaternion (no rotation)
        linear_velocity.setZero();
        angular_velocity.setZero();
    }

    // Packs the individual variables into a single 13D vector for the RK4 Integrator
    Eigen::VectorXd getState() const {
        Eigen::VectorXd state(13);
        state.segment<3>(0) = position;
        state.segment<4>(3) = orientation;
        state.segment<3>(7) = linear_velocity;
        state.segment<3>(10) = angular_velocity;
        return state;
    }

    // Unpacks the 13D vector back into the body properties
    void setState(const Eigen::VectorXd& state) {
        position = state.segment<3>(0);
        orientation = state.segment<4>(3);
        orientation.normalize(); // CRITICAL: Quaternions must remain unit length to represent pure rotation
        linear_velocity = state.segment<3>(7);
        angular_velocity = state.segment<3>(10);
    }

    // The core Kinematic Equations (Phase 1: Zero Force/Torque)
    Eigen::VectorXd computeStateDerivative() const {
        Eigen::VectorXd derivative(13);
        
        // 1. Position derivative is simply current linear velocity: r_dot = v
        derivative.segment<3>(0) = linear_velocity;
        
        // 2. Quaternion derivative: q_dot = 0.5 * Omega(w) * q
        Eigen::Matrix4d Omega = QuaternionHelper::buildOmegaMatrix(angular_velocity);
        derivative.segment<4>(3) = 0.5 * Omega * orientation;
        
        // 3. Linear velocity derivative: v_dot = 0 (No forces applied yet)
        derivative.segment<3>(7) = Eigen::Vector3d::Zero();
        
        // 4. Angular velocity derivative: w_dot = 0 (No torques applied yet)
        derivative.segment<3>(10) = Eigen::Vector3d::Zero();
        
        return derivative;
    }
};

} // namespace rbd