#include <cmath>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <iostream>
#include <ipn_mpc/control/dynamics_control_factor.h>

namespace {
bool near(double actual, double expected, double tolerance = 1.0e-6) {
    return std::abs(actual - expected) <= tolerance;
}
} // namespace

int main() {
    using gtsam::symbol_shorthand::S;
    using gtsam::symbol_shorthand::U;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;

    constexpr double dt = 0.1;
    UAVFactor::DynamicsParams params;
    const gtsam::Vector4 hover(params.mass * params.g, 0.0, 0.0, 0.0);
    const gtsam::KeyVector controls{U(0), U(1), U(2)};
    const gtsam::Matrix information = gtsam::Matrix::Identity(12, 12);
    const auto model = gtsam::noiseModel::Gaussian::Information(information);
    UAVFactor::TerminalDynamicsFactor factor(X(0), V(0), S(0), controls, X(3), V(3), S(3), dt,
                                               model, params);

    const UAVFactor::DynamicsState initial{gtsam::Pose3(), gtsam::Vector3::Zero(),
                                            gtsam::Vector3::Zero()};
    const std::vector<gtsam::Vector4> hover_controls(controls.size(), hover);
    const UAVFactor::DynamicsState hover_prediction = factor.predict(initial, hover_controls);
    if (hover_prediction.pose.translation().norm() > 1.0e-6 ||
        hover_prediction.velocity.norm() > 1.0e-6 || hover_prediction.body_rate.norm() > 1.0e-6) {
        std::cerr << "Hover rollout drifted: p=" << hover_prediction.pose.translation().transpose()
                  << ", v=" << hover_prediction.velocity.transpose()
                  << ", omega=" << hover_prediction.body_rate.transpose() << '\n';
        return 1;
    }

    gtsam::Values values;
    values.insert(X(0), initial.pose);
    values.insert(V(0), initial.velocity);
    values.insert(S(0), initial.body_rate);
    for (gtsam::Key key : controls) values.insert(key, hover);
    values.insert(X(3), hover_prediction.pose);
    values.insert(V(3), hover_prediction.velocity);
    values.insert(S(3), hover_prediction.body_rate);

    std::vector<gtsam::Matrix> jacobians;
    const gtsam::Vector error = factor.unwhitenedError(values, jacobians);
    if (error.size() != 12 || error.norm() > 1.0e-9) {
        std::cerr << "Exact terminal rollout has nonzero error: " << error.transpose() << '\n';
        return 2;
    }
    const int columns[] = {6, 3, 3, 4, 4, 4, 6, 3, 3};
    if (jacobians.size() != std::size(columns)) return 3;
    for (std::size_t i = 0; i < jacobians.size(); ++i) {
        if (jacobians[i].rows() != 12 || jacobians[i].cols() != columns[i] ||
            !jacobians[i].allFinite()) {
            std::cerr << "Invalid Jacobian at key index " << i << '\n';
            return 4;
        }
    }

    // One-step vertical acceleration checks the actual discrete dynamics formula.
    UAVFactor::TerminalDynamicsFactor one_step(X(0), V(0), S(0), {U(0)}, X(1), V(1), S(1), dt,
                                                 model, params);
    const auto accelerated = one_step.predict(
        initial, {gtsam::Vector4(2.0 * params.mass * params.g, 0.0, 0.0, 0.0)});
    if (!near(accelerated.pose.z(), 0.5 * params.g * dt * dt) ||
        !near(accelerated.velocity.z(), params.g * dt)) {
        std::cerr << "Vertical rollout does not match constant-acceleration dynamics\n";
        return 5;
    }

    // Verify that Q_j is used as the terminal residual information matrix.
    gtsam::Matrix weighted_information = gtsam::Matrix::Identity(12, 12);
    weighted_information(0, 0) = 25.0;
    UAVFactor::TerminalDynamicsFactor weighted_factor(
        X(0), V(0), S(0), controls, X(3), V(3), S(3), dt,
        gtsam::noiseModel::Gaussian::Information(weighted_information), params);
    values.update(X(3), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0.0, 0.0)));
    if (!near(weighted_factor.error(values), 0.5, 1.0e-7)) {
        std::cerr << "Q_j information weighting is incorrect\n";
        return 6;
    }

    // Acceleration/angular-speed factor: u = [a_world, omega_body].
    const gtsam::KeyVector kinematic_controls{U(0), U(1)};
    const auto kinematic_model = gtsam::noiseModel::Gaussian::Information(
        gtsam::Matrix::Identity(9, 9));
    UAVFactor::TerminalAccelerationGyroFactor kinematic_factor(
        X(0), V(0), kinematic_controls, X(2), V(2), dt, kinematic_model);
    gtsam::Vector6 kinematic_input;
    kinematic_input << 1.0, -2.0, 0.5, 0.0, 0.0, 0.3;
    const auto kinematic_prediction =
        kinematic_factor.predict(initial, {kinematic_input, kinematic_input});
    const double duration = 2.0 * dt;
    const gtsam::Vector3 expected_position =
        0.5 * kinematic_input.head<3>() * duration * duration;
    const gtsam::Vector3 expected_velocity = kinematic_input.head<3>() * duration;
    const double expected_yaw = kinematic_input(5) * duration;
    if ((kinematic_prediction.pose.translation() - expected_position).norm() > 1.0e-9 ||
        (kinematic_prediction.velocity - expected_velocity).norm() > 1.0e-9 ||
        std::abs(kinematic_prediction.pose.rotation().yaw() - expected_yaw) > 1.0e-9) {
        std::cerr << "Acceleration/angular-speed rollout is incorrect\n";
        return 7;
    }

    gtsam::Values kinematic_values;
    kinematic_values.insert(X(0), initial.pose);
    kinematic_values.insert(V(0), initial.velocity);
    for (gtsam::Key key : kinematic_controls) kinematic_values.insert(key, kinematic_input);
    kinematic_values.insert(X(2), kinematic_prediction.pose);
    kinematic_values.insert(V(2), kinematic_prediction.velocity);
    std::vector<gtsam::Matrix> kinematic_jacobians;
    const gtsam::Vector kinematic_error =
        kinematic_factor.unwhitenedError(kinematic_values, kinematic_jacobians);
    const int kinematic_columns[] = {6, 3, 6, 6, 6, 3};
    if (kinematic_error.norm() > 1.0e-9 ||
        kinematic_jacobians.size() != std::size(kinematic_columns)) {
        std::cerr << "Exact kinematic rollout has nonzero error\n";
        return 8;
    }
    for (std::size_t i = 0; i < kinematic_jacobians.size(); ++i) {
        if (kinematic_jacobians[i].rows() != 9 ||
            kinematic_jacobians[i].cols() != kinematic_columns[i] ||
            !kinematic_jacobians[i].allFinite()) {
            std::cerr << "Invalid kinematic Jacobian at key index " << i << '\n';
            return 9;
        }
    }

    // Measurement variant has no X(j) or V(j) variables: the predicted terminal state is
    // compared directly with a fixed pose/velocity set point.
    UAVFactor::TerminalAccelerationGyroMeasurementFactor measurement_factor(
        X(0), V(0), kinematic_controls, kinematic_prediction.pose,
        kinematic_prediction.velocity, dt, kinematic_model);
    gtsam::Values measurement_values;
    measurement_values.insert(X(0), initial.pose);
    measurement_values.insert(V(0), initial.velocity);
    for (gtsam::Key key : kinematic_controls)
        measurement_values.insert(key, kinematic_input);

    std::vector<gtsam::Matrix> measurement_jacobians;
    const gtsam::Vector measurement_error =
        measurement_factor.unwhitenedError(measurement_values, measurement_jacobians);
    const int measurement_columns[] = {6, 3, 6, 6};
    if (measurement_factor.keys().size() != 4 || measurement_error.norm() > 1.0e-9 ||
        measurement_jacobians.size() != std::size(measurement_columns)) {
        std::cerr << "Measurement factor contains a terminal state variable or has nonzero error\n";
        return 10;
    }
    for (std::size_t i = 0; i < measurement_jacobians.size(); ++i) {
        if (measurement_jacobians[i].rows() != 9 ||
            measurement_jacobians[i].cols() != measurement_columns[i] ||
            !measurement_jacobians[i].allFinite()) {
            std::cerr << "Invalid measurement-factor Jacobian at key index " << i << '\n';
            return 11;
        }
    }

    const gtsam::Pose3 offset_set_point(
        kinematic_prediction.pose.rotation(),
        kinematic_prediction.pose.translation() + gtsam::Vector3(0.2, 0.0, 0.0));
    gtsam::Matrix measurement_information = gtsam::Matrix::Identity(9, 9);
    measurement_information(0, 0) = 25.0;
    UAVFactor::TerminalAccelerationGyroMeasurementFactor weighted_measurement_factor(
        X(0), V(0), kinematic_controls, offset_set_point, kinematic_prediction.velocity, dt,
        gtsam::noiseModel::Gaussian::Information(measurement_information));
    if (!near(weighted_measurement_factor.error(measurement_values), 0.5, 1.0e-7)) {
        std::cerr << "Measurement set-point Q_j weighting is incorrect\n";
        return 12;
    }

    std::cout << "Terminal dynamics factor tests passed\n";
    return 0;
}
