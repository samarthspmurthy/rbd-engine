#pragma once
#include <Eigen/Dense>

namespace rbd {

class InertiaTensor {
public:
    // Solid Box Inertia
    static Eigen::Matrix3d createSolidBox(double mass, double dx, double dy, double dz) {
        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        I(0, 0) = (1.0 / 12.0) * mass * (dy * dy + dz * dz);
        I(1, 1) = (1.0 / 12.0) * mass * (dx * dx + dz * dz);
        I(2, 2) = (1.0 / 12.0) * mass * (dx * dx + dy * dy);
        return I;
    }

    // Slender Rod aligned with the X-axis
    static Eigen::Matrix3d createSlenderRodX(double mass, double length) {
        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        // Ixx is roughly 0 for a theoretically infinitely thin rod
        I(1, 1) = (1.0 / 12.0) * mass * (length * length);
        I(2, 2) = (1.0 / 12.0) * mass * (length * length);
        return I;
    }

    // Parallel Axis Theorem: I_new = I_cm + m * (r^2 * I_3x3 - r * r^T)
    static Eigen::Matrix3d applyParallelAxisTheorem(const Eigen::Matrix3d& I_cm, double mass, const Eigen::Vector3d& offset) {
        double x = offset.x(), y = offset.y(), z = offset.z();
        Eigen::Matrix3d offset_matrix;
        offset_matrix << (y*y + z*z), -x*y, -x*z,
                         -x*y, (x*x + z*z), -y*z,
                         -x*z, -y*z, (x*x + y*y);
        return I_cm + mass * offset_matrix;
    }
};

} // namespace rbd