#pragma once
#include <Eigen/Dense>

namespace rbd {

class QuaternionHelper {
public:
    // Builds the 4x4 Omega matrix from body-frame angular velocity (w)
    // Matches Blueprint Section 1.3 for quaternion kinematics
    static Eigen::Matrix4d buildOmegaMatrix(const Eigen::Vector3d& w) {
        Eigen::Matrix4d Omega;
        Omega <<  0,     -w.x(), -w.y(), -w.z(),
                  w.x(),  0,      w.z(), -w.y(),
                  w.y(), -w.z(),  0,      w.x(),
                  w.z(),  w.y(), -w.x(),  0;
        return Omega;
    }
};

} // namespace rbd