#pragma once
#include <Eigen/Dense>

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
    double inverse_mass; 
    Eigen::Matrix3d inertia_tensor;
    Eigen::Matrix3d inverse_inertia_tensor;

    // Force and Torque accumulators
    Eigen::Vector3d force_accumulator;
    Eigen::Vector3d torque_accumulator;

    RigidBody();

    void setMass(double m);
    void setInertiaTensor(const Eigen::Matrix3d& I);
    
    // Physics Inputs
    void applyForce(const Eigen::Vector3d& force);
    void applyTorque(const Eigen::Vector3d& torque);
    void clearAccumulators();

    // State packing/unpacking
    Eigen::VectorXd getState() const;
    void setState(const Eigen::VectorXd& state);

    // Kinematics + Dynamics
    Eigen::VectorXd computeStateDerivative() const;
};

} // namespace rbd