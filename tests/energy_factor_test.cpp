#include <cmath>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <iostream>
#include <ipn_mpc/control/energy_control_factor.h>
#include <stdexcept>
#include <vector>

namespace {
constexpr double kTolerance = 1.0e-6;

bool close(const gtsam::Matrix& actual, const gtsam::Matrix& expected,
           const char* quantity) {
    if (actual.rows() == expected.rows() && actual.cols() == expected.cols() &&
        (actual - expected).norm() <= kTolerance)
        return true;
    std::cerr << quantity << " mismatch\nactual:\n"
              << actual << "\nexpected:\n" << expected << '\n';
    return false;
}
} // namespace

int main() {
    using gtsam::symbol_shorthand::T;
    using gtsam::symbol_shorthand::U;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;

    const gtsam::Vector3 drag_k(-0.20, -0.30, -0.40);
    const gtsam::Vector3 power_curve(2.0, 0.5, 0.1);
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const EnergyFactor factor(X(0), V(0), U(0), T(0), drag_k, power_curve, noise);

    const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.2, -0.3, 0.4),
                            gtsam::Point3(4.0, -2.0, 1.0));
    const gtsam::Vector3 velocity(3.0, -1.0, 0.5);
    const gtsam::Vector4 input(1.0, 2.0, 3.0, 4.0);
    constexpr double dt = 0.25;

    gtsam::Matrix H_pose, H_velocity, H_input, H_dt;
    const gtsam::Vector error =
        factor.evaluateError(pose, velocity, input, dt, &H_pose, &H_velocity, &H_input, &H_dt);

    const gtsam::Vector3 body_velocity = pose.rotation().unrotate(velocity);
    const double drag_power = -body_velocity.dot(drag_k.cwiseProduct(body_velocity));
    const double actuator_power =
        (gtsam::Vector4::Constant(power_curve(0)).array() +
         power_curve(1) * input.array() + power_curve(2) * input.array().square()).sum();
    const double expected_energy = dt * (actuator_power + drag_power);
    if (std::abs(error(0) - expected_energy) > kTolerance) {
        std::cerr << "Energy residual mismatch: " << error(0) << " vs " << expected_energy << '\n';
        return 1;
    }

    gtsam::Values values;
    values.insert(X(0), pose);
    values.insert(V(0), velocity);
    values.insert(U(0), input);
    values.insert(T(0), static_cast<float>(dt));
    std::vector<gtsam::Matrix> graph_jacobians(4);
    const gtsam::Vector graph_error = factor.unwhitenedError(values, &graph_jacobians);
    if (std::abs(graph_error(0) - expected_energy) > kTolerance ||
        graph_jacobians[0].cols() != 6 || graph_jacobians[1].cols() != 3 ||
        graph_jacobians[2].cols() != 4 || graph_jacobians[3].cols() != 1) {
        std::cerr << "GTSAM factor dispatch returned an invalid residual or Jacobian shape\n";
        return 2;
    }

    const auto evaluate = [&](const gtsam::Pose3& p, const gtsam::Vector3& v,
                              const gtsam::Vector4& u, double interval) {
        return factor.evaluateError(p, v, u, interval);
    };
    const gtsam::Matrix numerical_pose =
        gtsam::numericalDerivative41<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                     gtsam::Vector4, double>(evaluate, pose, velocity, input, dt);
    const gtsam::Matrix numerical_velocity =
        gtsam::numericalDerivative42<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                     gtsam::Vector4, double>(evaluate, pose, velocity, input, dt);
    const gtsam::Matrix numerical_input =
        gtsam::numericalDerivative43<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                     gtsam::Vector4, double>(evaluate, pose, velocity, input, dt);
    const gtsam::Matrix numerical_dt =
        gtsam::numericalDerivative44<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                     gtsam::Vector4, double>(evaluate, pose, velocity, input, dt);

    if (!close(H_pose, numerical_pose, "pose Jacobian") ||
        !close(H_velocity, numerical_velocity, "velocity Jacobian") ||
        !close(H_input, numerical_input, "input Jacobian") ||
        !close(H_dt, numerical_dt, "dt Jacobian"))
        return 3;

    bool rejected_bad_noise = false;
    try {
        EnergyFactor invalid(X(0), V(0), U(0), T(0), drag_k, power_curve,
                             gtsam::noiseModel::Isotropic::Sigma(2, 1.0));
    } catch (const std::invalid_argument&) {
        rejected_bad_noise = true;
    }
    if (!rejected_bad_noise) {
        std::cerr << "EnergyFactor accepted a non-scalar noise model\n";
        return 4;
    }

    std::cout << "Energy factor residual and Jacobian tests passed\n";
    return 0;
}
