#include "rbd/TwoLinkArm.hpp"
#include <cmath>

namespace rbd {

TwoLinkArm::TwoLinkArm() : m1(1), m2(1), l1(1), l2(1), lc1(0.5), lc2(0.5), I1(0.1), I2(0.1), g(9.81) {
    state.setZero();
}

Eigen::Vector4d TwoLinkArm::computeDerivatives(const Eigen::Vector4d& current_state, const Eigen::Vector2d& tau) {
    double t1 = current_state(0);
    double t2 = current_state(1);
    double t1_dot = current_state(2);
    double t2_dot = current_state(3);

    // 1. Mass Matrix M(theta)
    Eigen::Matrix2d M;
    M(0, 0) = I1 + I2 + m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * cos(t2));
    M(0, 1) = I2 + m2 * (lc2 * lc2 + l1 * lc2 * cos(t2));
    M(1, 0) = M(0, 1);
    M(1, 1) = I2 + m2 * lc2 * lc2;

    // 2. Coriolis/Centrifugal Matrix C(theta, theta_dot)
    double h = -m2 * l1 * lc2 * sin(t2);
    Eigen::Matrix2d C;
    C(0, 0) = h * t2_dot;
    C(0, 1) = h * (t1_dot + t2_dot);
    C(1, 0) = -h * t1_dot;
    C(1, 1) = 0.0;

    // 3. Gravity Vector G(theta)
    Eigen::Vector2d G;
    G(0) = (m1 * lc1 + m2 * l1) * g * cos(t1) + m2 * lc2 * g * cos(t1 + t2);
    G(1) = m2 * lc2 * g * cos(t1 + t2);

    // 4. Solve for accelerations: theta_ddot = M^-1 * (tau - C*theta_dot - G)
    Eigen::Vector2d theta_dot(t1_dot, t2_dot);
    Eigen::Vector2d theta_ddot = M.inverse() * (tau - C * theta_dot - G);

    // Return state derivative [theta1_dot, theta2_dot, theta1_ddot, theta2_ddot]
    Eigen::Vector4d state_derivative;
    state_derivative << t1_dot, t2_dot, theta_ddot(0), theta_ddot(1);
    return state_derivative;
}

double TwoLinkArm::getTotalEnergy() {
    double t1 = state(0), t2 = state(1);
    Eigen::Vector2d theta_dot(state(2), state(3));

    // Recompute M for Kinetic Energy
    Eigen::Matrix2d M;
    M(0, 0) = I1 + I2 + m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * cos(t2));
    M(0, 1) = I2 + m2 * (lc2 * lc2 + l1 * lc2 * cos(t2));
    M(1, 0) = M(0, 1);
    M(1, 1) = I2 + m2 * lc2 * lc2;

    double kinetic = 0.5 * theta_dot.transpose() * M * theta_dot;
    double potential = m1 * g * (lc1 * sin(t1)) + m2 * g * (l1 * sin(t1) + lc2 * sin(t1 + t2));

    return kinetic + potential;
}

} // namespace rbd