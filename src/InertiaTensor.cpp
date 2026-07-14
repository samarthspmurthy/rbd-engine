#include "rbd/InertiaTensor.hpp"

namespace rbd {

Eigen::Matrix3d InertiaTensor::createSolidBox(double mass, double dx, double dy, double dz) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    I(0, 0) = (1.0 / 12.0) * mass * (dy * dy + dz * dz);
    I(1, 1) = (1.0 / 12.0) * mass * (dx * dx + dz * dz);
    I(2, 2) = (1.0 / 12.0) * mass * (dx * dx + dy * dy);
    return I;
}

Eigen::Matrix3d InertiaTensor::createSlenderRodX(double mass, double length) {
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
    // Ixx is roughly 0 for a theoretically infinitely thin rod
    I(1, 1) = (1.0 / 12.0) * mass * (length * length);
    I(2, 2) = (1.0 / 12.0) * mass * (length * length);
    return I;
}

Eigen::Matrix3d InertiaTensor::applyParallelAxisTheorem(const Eigen::Matrix3d& I_cm, double mass, const Eigen::Vector3d& offset) {
    double x = offset.x(), y = offset.y(), z = offset.z();
    Eigen::Matrix3d offset_matrix;
    offset_matrix << (y*y + z*z), -x*y, -x*z,
                     -x*y, (x*x + z*z), -y*z,
                     -x*z, -y*z, (x*x + y*y);
    return I_cm + mass * offset_matrix;
}

} // namespace rbd