#include <cmath>
#include <gtsam/config.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <iostream>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <vector>

int main() {
    static_assert(GTSAM_VERSION_NUMERIC >= 40300, "IPN_MPC requires GTSAM 4.3 or newer");

    using gtsam::symbol_shorthand::S;
    using gtsam::symbol_shorthand::U;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;

    constexpr double mass = 0.98;
    constexpr double gravity = 9.81;

    const auto noise = gtsam::noiseModel::Isotropic::Sigma(12, 1.0);
    UAVFactor::DynamicsFactorTm factor(X(0), V(0), S(0), U(0), X(1), V(1), S(1), 0.1F, noise);

    const gtsam::Vector3 zero = gtsam::Vector3::Zero();
    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3());
    values.insert(V(0), zero);
    values.insert(S(0), zero);
    values.insert(U(0), gtsam::Vector4(mass * gravity, 0.0, 0.0, 0.0));
    values.insert(X(1), gtsam::Pose3());
    values.insert(V(1), zero);
    values.insert(S(1), zero);

    std::vector<gtsam::Matrix> jacobians(7);
    const gtsam::Vector error = factor.unwhitenedError(values, &jacobians);

    if (error.size() != 12 || error.norm() > 1e-7) {
        std::cerr << "Hover-state dynamics error is not zero: " << error.transpose() << '\n';
        return 1;
    }

    const int expected_columns[] = {6, 3, 3, 4, 6, 3, 3};
    for (std::size_t i = 0; i < jacobians.size(); ++i) {
        if (jacobians[i].rows() != 12 || jacobians[i].cols() != expected_columns[i]) {
            std::cerr << "Unexpected Jacobian dimensions at index " << i << '\n';
            return 2;
        }
        if (!jacobians[i].allFinite()) {
            std::cerr << "Non-finite Jacobian at index " << i << '\n';
            return 3;
        }
    }

    std::cout << "GTSAM " << GTSAM_VERSION_STRING << " compatibility test passed\n";
    return 0;
}
