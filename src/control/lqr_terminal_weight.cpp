#include <Eigen/Eigenvalues>
#include <cmath>
#include <ipn_mpc/control/lqr_terminal_weight.h>
#include <stdexcept>

namespace UAVFactor {
namespace {

void validatePositiveDefinite(const gtsam::Matrix& matrix, const char* name) {
    if (!matrix.allFinite() || !matrix.isApprox(matrix.transpose(), 1.0e-12) ||
        Eigen::LLT<gtsam::Matrix>(matrix).info() != Eigen::Success)
        throw std::invalid_argument(std::string(name) + " must be symmetric positive definite");
}

} // namespace

LqrTerminalWeight computeAccelerationGyroTerminalWeight(
    double dt, const gtsam::Matrix9& state_information,
    const gtsam::Matrix6& control_information, double tolerance,
    std::size_t maximum_iterations) {
    if (dt <= 0.0 || tolerance <= 0.0 || maximum_iterations == 0)
        throw std::invalid_argument("dt, tolerance, and maximum_iterations must be positive");
    validatePositiveDefinite(state_information, "state_information");
    validatePositiveDefinite(control_information, "control_information");

    // x = [p, theta, v], u = [a, omega]. This is the local discrete-time model used by
    // TerminalAccelerationGyroMeasurementFactor around zero tracking error.
    gtsam::Matrix9 A = gtsam::Matrix9::Identity();
    A.block<3, 3>(0, 6) = dt * gtsam::Matrix3::Identity();
    gtsam::Matrix96 B = gtsam::Matrix96::Zero();
    B.block<3, 3>(0, 0) = 0.5 * dt * dt * gtsam::Matrix3::Identity();
    B.block<3, 3>(3, 3) = dt * gtsam::Matrix3::Identity();
    B.block<3, 3>(6, 0) = dt * gtsam::Matrix3::Identity();

    gtsam::Matrix9 P = state_information;
    std::size_t iterations = 0;
    for (; iterations < maximum_iterations; ++iterations) {
        const gtsam::Matrix6 hessian = control_information + B.transpose() * P * B;
        const gtsam::Matrix69 gain =
            hessian.ldlt().solve(B.transpose() * P * A);
        gtsam::Matrix9 next = state_information + A.transpose() * P * A -
                              A.transpose() * P * B * gain;
        next = 0.5 * (next + next.transpose());
        const double relative_change =
            (next - P).norm() / std::max(1.0, next.norm());
        P = next;
        if (relative_change < tolerance) {
            ++iterations;
            break;
        }
    }
    if (iterations == maximum_iterations)
        throw std::runtime_error("DARE terminal-weight iteration did not converge");

    const gtsam::Matrix6 hessian = control_information + B.transpose() * P * B;
    const gtsam::Matrix69 gain = hessian.ldlt().solve(B.transpose() * P * A);
    const gtsam::Matrix9 riccati_right =
        state_information + A.transpose() * P * A - A.transpose() * P * B * gain;
    const double residual = (P - riccati_right).norm() / std::max(1.0, P.norm());

    const gtsam::Matrix9 closed_loop = A - B * gain;
    const Eigen::EigenSolver<gtsam::Matrix9> eigen_solver(closed_loop, false);
    double spectral_radius = 0.0;
    for (Eigen::Index i = 0; i < eigen_solver.eigenvalues().size(); ++i)
        spectral_radius =
            std::max(spectral_radius, std::abs(eigen_solver.eigenvalues()(i)));
    if (!std::isfinite(spectral_radius) || spectral_radius >= 1.0)
        throw std::runtime_error("Computed DARE feedback is not asymptotically stable");

    return {P, gain, residual, spectral_radius, iterations};
}

} // namespace UAVFactor
