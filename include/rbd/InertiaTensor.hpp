#pragma once
#include <Eigen/Dense>

namespace rbd {

class InertiaTensor {
public:
    // Solid Box Inertia
    static Eigen::Matrix3d createSolidBox(double mass, double dx, double dy, double dz);

    // Slender Rod aligned with the X-axis
    static Eigen::Matrix3d createSlenderRodX(double mass, double length);

    // Parallel Axis Theorem: I_new = I_cm + m * (r^2 * I_3x3 - r * r^T)
    static Eigen::Matrix3d applyParallelAxisTheorem(const Eigen::Matrix3d& I_cm, double mass, const Eigen::Vector3d& offset);
};

} // namespace rbd