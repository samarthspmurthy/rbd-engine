#pragma once
#include <Eigen/Dense>
#include "Quaternion.hpp"

namespace rbd {

class RigidBody {
public:
    // 13-Dimensional State
    Eigen::Vector3d position;
    Eigen::Vector4d orientation;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    // --- PHASE 2 ADDITIONS: Mass & Inertia Properties ---
    double mass;
    double inverse_mass; // Stored to avoid division during the RK4 loop
    Eigen::Matrix3d inertia_tensor;
    Eigen::Matrix3d inverse_inertia_tensor;

    // Force and Torque accumulators
    Eigen::Vector3d force_accumulator;
    Eigen::Vector3d torque_accumulator;

    RigidBody() {
        position.setZero();
        orientation << 1.0, 0.0, 0.0, 0.0;
        linear_velocity.setZero();
        angular_velocity.setZero();

        // Default to a 1kg object with a generic identity inertia matrix
        setMass(1.0);
        setInertiaTensor(Eigen::Matrix3d::Identity());
        
        clearAccumulators();
    }

    // Setters compute the inverses immediately to save CPU cycles later
    void setMass(double m) {
        mass = m;
        inverse_mass = (m > 0.0) ? (1.0 / m) : 0.0;
    }

    void setInertiaTensor(const Eigen::Matrix3d& I) {
        inertia_tensor = I;
        inverse_inertia_tensor = I.inverse();
    }

    // Physics Inputs
    void applyForce(const Eigen::Vector3d& force) {
        force_accumulator += force;
    }

    void applyTorque(const Eigen::Vector3d& torque) {
        torque_accumulator += torque;
    }

    void clearAccumulators() {
        force_accumulator.setZero();
        torque_accumulator.setZero();
    }

    // State packing/unpacking (Unchanged from Phase 1)
    Eigen::VectorXd getState() const {
        Eigen::VectorXd state(13);
        state.segment<3>(0) = position;
        state.segment<4>(3) = orientation;
        state.segment<3>(7) = linear_velocity;
        state.segment<3>(10) = angular_velocity;
        return state;
    }

    void setState(const Eigen::VectorXd& state) {
        position = state.segment<3>(0);
        orientation = state.segment<4>(3);
        orientation.normalize(); 
        linear_velocity = state.segment<3>(7);
        angular_velocity = state.segment<3>(10);
    }

    // --- PHASE 2 UPDATE: Kinematics + Dynamics ---
    Eigen::VectorXd computeStateDerivative() const {
        Eigen::VectorXd derivative(13);
        
        // 1. Kinematics: r_dot = v
        derivative.segment<3>(0) = linear_velocity;
        
        // 2. Kinematics: q_dot = 0.5 * Omega(w) * q
        Eigen::Matrix4d Omega = QuaternionHelper::buildOmegaMatrix(angular_velocity);
        derivative.segment<4>(3) = 0.5 * Omega * orientation;
        
        // 3. Dynamics: v_dot = F / m
        derivative.segment<3>(7) = force_accumulator * inverse_mass;
        
        // 4. Dynamics: w_dot = I^-1 * (tau - w x (I * w))
        Eigen::Vector3d gyroscopic_torque = angular_velocity.cross(inertia_tensor * angular_velocity);
        derivative.segment<3>(10) = inverse_inertia_tensor * (torque_accumulator - gyroscopic_torque);
        
        return derivative;
    }
};

} // namespace rbd