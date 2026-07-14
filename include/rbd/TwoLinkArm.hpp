#pragma once
#include <Eigen/Dense>

namespace rbd {

class TwoLinkArm {
public:
    // Parameters
    double m1, m2;
    double l1, l2;
    double lc1, lc2;
    double I1, I2;
    double g;

    // State Vector: [theta1, theta2, theta1_dot, theta2_dot]
    Eigen::Vector4d state;

    TwoLinkArm();

    // The state-space f(x,t) function
    Eigen::Vector4d computeDerivatives(const Eigen::Vector4d& current_state, const Eigen::Vector2d& tau);

    // Energy Calculation for Validation
    double getTotalEnergy();
};

} // namespace rbd