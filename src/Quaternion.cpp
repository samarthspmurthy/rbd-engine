#include "rbd/Quaternion.hpp"

namespace rbd {

Eigen::Matrix4d QuaternionHelper::buildOmegaMatrix(const Eigen::Vector3d& w) {
    Eigen::Matrix4d Omega;
    Omega <<  0,     -w.x(), -w.y(), -w.z(),
              w.x(),  0,      w.z(), -w.y(),
              w.y(), -w.z(),  0,      w.x(),
              w.z(),  w.y(), -w.x(),  0;
    return Omega;
}

} // namespace rbd