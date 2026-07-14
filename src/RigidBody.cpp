#include "rbd/RigidBody.hpp"
#include "rbd/Quaternion.hpp"

namespace rbd {

RigidBody::RigidBody() {
    position.setZero();
    orientation << 1.0, 0.0, 0.0, 0.0;
    linear_velocity.setZero();
    angular_velocity.setZero();

    // Default to a 1kg object with a generic identity inertia matrix
    setMass(1.0);
    setInertiaTensor(Eigen::Matrix3d::Identity());
    
    clearAccumulators();
}

void RigidBody::setMass(double m) {
    mass = m;
    inverse_mass = (m > 0.0) ? (1.0 / m) : 0.0;
}

void RigidBody::setInertiaTensor(const Eigen::Matrix3d& I) {
    inertia_tensor = I;
    inverse_inertia_tensor = I.inverse();
}

void RigidBody::applyForce(const Eigen::Vector3d& force) {
    force_accumulator += force;
}

void RigidBody::applyTorque(const Eigen::Vector3d& torque) {
    torque_accumulator += torque;
}

void RigidBody::clearAccumulators() {
    force_accumulator.setZero();
    torque_accumulator.setZero();
}

Eigen::VectorXd RigidBody::getState() const {
    Eigen::VectorXd state(13);
    state.segment<3>(0) = position;
    state.segment<4>(3) = orientation;
    state.segment<3>(7) = linear_velocity;
    state.segment<3>(10) = angular_velocity;
    return state;
}

void RigidBody::setState(const Eigen::VectorXd& state) {
    position = state.segment<3>(0);
    orientation = state.segment<4>(3);
    orientation.normalize(); 
    linear_velocity = state.segment<3>(7);
    angular_velocity = state.segment<3>(10);
}

Eigen::VectorXd RigidBody::computeStateDerivative() const {
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

} // namespace rbd