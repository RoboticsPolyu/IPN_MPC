#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <ipn_mpc/control/lqr_terminal_weight.h>

int main() {
    constexpr double dt = 0.1;
    gtsam::Matrix9 Q = gtsam::Matrix9::Zero();
    Q.diagonal() << gtsam::Vector3::Constant(1.0 / (0.02 * 0.02)),
        gtsam::Vector3::Constant(1.0 / (0.02 * 0.02)),
        gtsam::Vector3::Constant(1.0 / (0.05 * 0.05));
    gtsam::Matrix6 R = gtsam::Matrix6::Zero();
    R.diagonal() << gtsam::Vector3::Constant(1.0 / (5.0 * 5.0)),
        gtsam::Vector3::Constant(1.0 / (3.0 * 3.0));

    const UAVFactor::LqrTerminalWeight result =
        UAVFactor::computeAccelerationGyroTerminalWeight(dt, Q, R);
    if (result.information.rows() != 9 || result.information.cols() != 9 ||
        result.feedback_gain.rows() != 6 || result.feedback_gain.cols() != 9 ||
        !result.information.allFinite() || !result.feedback_gain.allFinite()) {
        std::cerr << "Invalid terminal-weight dimensions or values\n";
        return 1;
    }
    if (result.riccati_residual > 1.0e-10 ||
        result.closed_loop_spectral_radius >= 1.0) {
        std::cerr << "Terminal weight is not a stabilizing DARE solution: residual="
                  << result.riccati_residual
                  << " spectral_radius=" << result.closed_loop_spectral_radius << '\n';
        return 2;
    }

    gtsam::Matrix9 A = gtsam::Matrix9::Identity();
    A.block<3, 3>(0, 6) = dt * gtsam::Matrix3::Identity();
    gtsam::Matrix96 B = gtsam::Matrix96::Zero();
    B.block<3, 3>(0, 0) = 0.5 * dt * dt * gtsam::Matrix3::Identity();
    B.block<3, 3>(3, 3) = dt * gtsam::Matrix3::Identity();
    B.block<3, 3>(6, 0) = dt * gtsam::Matrix3::Identity();
    const gtsam::Matrix9 closed_loop = A - B * result.feedback_gain;
    const gtsam::Matrix9 lyapunov_change =
        closed_loop.transpose() * result.information * closed_loop - result.information;
    const Eigen::SelfAdjointEigenSolver<gtsam::Matrix9> lyapunov_solver(lyapunov_change);
    if (lyapunov_solver.eigenvalues().maxCoeff() >= -1.0e-8) {
        std::cerr << "Terminal cost does not strictly decrease under the LQR feedback\n";
        return 3;
    }

    const Eigen::SelfAdjointEigenSolver<gtsam::Matrix9> dominance_solver(
        result.information - Q);
    if (dominance_solver.eigenvalues().minCoeff() < -1.0e-8) {
        std::cerr << "Terminal information does not dominate the stage-state information\n";
        return 4;
    }

    std::cout << "DARE residual=" << result.riccati_residual
              << " closed-loop spectral radius=" << result.closed_loop_spectral_radius
              << " iterations=" << result.iterations << "\nP=\n"
              << result.information << '\n';
    return 0;
}
