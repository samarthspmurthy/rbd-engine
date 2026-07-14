#pragma once
#include <Eigen/Dense>

namespace rbd {

class QuaternionHelper {
public:
    // Builds the 4x4 Omega matrix from body-frame angular velocity (w)
    // Matches Blueprint Section 1.3 for quaternion kinematics
    static Eigen::Matrix4d buildOmegaMatrix(const Eigen::Vector3d& w);
};

} // namespace rbd