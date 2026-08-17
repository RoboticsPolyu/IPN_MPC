#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <iostream>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <ipn_mpc/control/lqr_terminal_weight.h>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace {
gtsam::Vector3 readVector3(const YAML::Node& config, const char* key) {
    const YAML::Node value = config[key];
    if (!value || !value.IsSequence() || value.size() != 3)
        throw std::invalid_argument(std::string(key) + " must contain three values");
    return {value[0].as<double>(), value[1].as<double>(), value[2].as<double>()};
}

gtsam::Vector6 readVector6(const YAML::Node& config, const char* key) {
    const YAML::Node value = config[key];
    if (!value || !value.IsSequence() || value.size() != 6)
        throw std::invalid_argument(std::string(key) + " must contain six values");
    gtsam::Vector6 vector;
    for (std::size_t i = 0; i < 6; ++i) vector(i) = value[i].as<double>();
    return vector;
}

gtsam::Values enforceControlBounds(const gtsam::NonlinearFactorGraph& graph,
                                   const gtsam::Values& initial_values,
                                   const gtsam::KeyVector& control_keys,
                                   const gtsam::Vector6& control_min,
                                   const gtsam::Vector6& control_max,
                                   std::size_t sqp_iterations) {
    gtsam::Values values = initial_values;
    for (std::size_t iteration = 0; iteration < sqp_iterations; ++iteration) {
        const auto linear_cost = graph.linearize(values);
        gtsam::InequalityFactorGraph inequalities;
        gtsam::VectorValues feasible_delta = values.zeroVectors();
        std::size_t constraint_index = 0;

        for (gtsam::Key key : control_keys) {
            const gtsam::Vector6 control = values.at<gtsam::Vector6>(key);
            feasible_delta.at(key) =
                control.cwiseMax(control_min).cwiseMin(control_max) - control;
            for (Eigen::Index component = 0; component < 6; ++component) {
                gtsam::RowVector upper = gtsam::RowVector::Zero(6);
                upper(component) = 1.0;
                inequalities.add(key, upper, control_max(component) - control(component),
                                 gtsam::Symbol('q', constraint_index++));

                gtsam::RowVector lower = gtsam::RowVector::Zero(6);
                lower(component) = -1.0;
                inequalities.add(key, lower, control(component) - control_min(component),
                                 gtsam::Symbol('q', constraint_index++));
            }
        }

        const gtsam::QP problem(*linear_cost, gtsam::EqualityFactorGraph{}, inequalities);
        const gtsam::VectorValues delta =
            gtsam::QPSolver(problem).optimize(feasible_delta).first;
        values = values.retract(delta);
    }
    return values;
}

double maximumControlViolation(const gtsam::Values& values,
                               const gtsam::KeyVector& control_keys,
                               const gtsam::Vector6& control_min,
                               const gtsam::Vector6& control_max) {
    double maximum_violation = 0.0;
    for (gtsam::Key key : control_keys) {
        const gtsam::Vector6 control = values.at<gtsam::Vector6>(key);
        maximum_violation = std::max(
            maximum_violation,
            std::max((control_min - control).maxCoeff(), (control - control_max).maxCoeff()));
    }
    return std::max(0.0, maximum_violation);
}

gtsam::Vector6 controlSigmas(double acceleration_sigma, double angular_speed_sigma) {
    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3::Constant(acceleration_sigma),
        gtsam::Vector3::Constant(angular_speed_sigma);
    return sigmas;
}

double rotationError(const gtsam::Rot3& actual, const gtsam::Rot3& target) {
    return gtsam::Rot3::Logmap(actual.between(target)).norm();
}

UAVFactor::DynamicsState circleReference(const gtsam::Vector3& center, double radius,
                                         double angular_speed, double time, bool face_velocity) {
    const double angle = angular_speed * time;
    UAVFactor::DynamicsState reference;
    reference.pose = gtsam::Pose3(
        gtsam::Rot3::Rz(face_velocity ? angle + M_PI_2 : 0.0),
        center + gtsam::Vector3(radius * std::cos(angle), radius * std::sin(angle), 0.0));
    reference.velocity = gtsam::Vector3(-radius * angular_speed * std::sin(angle),
                                        radius * angular_speed * std::cos(angle), 0.0);
    reference.body_rate = gtsam::Vector3(0.0, 0.0, face_velocity ? angular_speed : 0.0);
    return reference;
}

gtsam::Rot3 thrustAlignedRotation(const gtsam::Vector3& force, double yaw) {
    const gtsam::Vector3 body_z = force.normalized();
    const gtsam::Vector3 heading(std::cos(yaw), std::sin(yaw), 0.0);
    gtsam::Vector3 body_y = body_z.cross(heading);
    if (body_y.norm() < 1.0e-8) body_y = gtsam::Vector3::UnitY();
    body_y.normalize();
    const gtsam::Vector3 body_x = body_y.cross(body_z).normalized();
    gtsam::Matrix3 rotation;
    rotation.col(0) = body_x;
    rotation.col(1) = body_y;
    rotation.col(2) = body_z;
    return gtsam::Rot3(rotation);
}

struct TimingStatistics {
    std::vector<double> samples;

    void add(double value) { samples.push_back(value); }
    double mean() const {
        if (samples.empty()) return 0.0;
        double sum = 0.0;
        for (double value : samples) sum += value;
        return sum / samples.size();
    }
    double standardDeviation() const {
        if (samples.size() < 2) return 0.0;
        const double average = mean();
        double sum = 0.0;
        for (double value : samples) sum += (value - average) * (value - average);
        return std::sqrt(sum / (samples.size() - 1));
    }
    double percentile(double probability) const {
        if (samples.empty()) return 0.0;
        std::vector<double> sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        const std::size_t index = static_cast<std::size_t>(
            std::ceil(probability * sorted.size()) - 1.0);
        return sorted[std::min(index, sorted.size() - 1)];
    }
    double minimum() const {
        return samples.empty() ? 0.0 : *std::min_element(samples.begin(), samples.end());
    }
    double maximum() const {
        return samples.empty() ? 0.0 : *std::max_element(samples.begin(), samples.end());
    }
};
} // namespace

int main(int argc, char** argv) {
    using gtsam::symbol_shorthand::U;
    using gtsam::symbol_shorthand::V;
    using gtsam::symbol_shorthand::X;

    try {
        const std::string config_path =
            argc > 1 ? argv[1] : "../config/terminal_acceleration_gyro_mpc.yaml";
        const YAML::Node config = YAML::LoadFile(config_path);
        const std::size_t horizon = config["horizon"].as<std::size_t>();
        const double dt = config["dt"].as<double>();
        const std::size_t mpc_iterations = config["mpc_iterations"].as<std::size_t>();
        const bool force_headless = argc > 2 && std::string(argv[2]) == "--headless";
        const bool visualize = config["visualize"].as<bool>() && !force_headless;
        const bool real_time_playback = config["real_time_playback"].as<bool>();
        const std::size_t statistics_warmup =
            config["statistics_warmup_iterations"].as<std::size_t>();
        const std::size_t statistics_print_interval =
            config["statistics_print_interval"].as<std::size_t>();
        const gtsam::Vector6 control_min = readVector6(config, "control_min");
        const gtsam::Vector6 control_max = readVector6(config, "control_max");
        const std::size_t hard_control_constraint_iterations =
            config["hard_control_constraint_iterations"].as<std::size_t>();
        if (horizon == 0 || dt <= 0.0 || mpc_iterations == 0 ||
            config["simulator_substeps"].as<std::size_t>() == 0 ||
            hard_control_constraint_iterations == 0)
            throw std::invalid_argument(
                "horizon, dt, mpc_iterations, simulator_substeps, and "
                "hard_control_constraint_iterations must be positive");
        if ((control_min.array() >= control_max.array()).any())
            throw std::invalid_argument("Every control_min component must be below control_max");

        UAVFactor::DynamicsState state{
            gtsam::Pose3(gtsam::Rot3::RzRyRx(readVector3(config, "initial_rpy")),
                         readVector3(config, "initial_position")),
            readVector3(config, "initial_velocity"), gtsam::Vector3::Zero()};
        const gtsam::Vector3 circle_center = readVector3(config, "circle_center");
        const double circle_radius = config["circle_radius"].as<double>();
        const double circle_angular_speed = config["circle_angular_speed"].as<double>();
        const bool face_velocity = config["face_velocity"].as<bool>();
        const std::filesystem::path simulator_config =
            std::filesystem::path(config_path).parent_path() /
            config["quadrotor_config"].as<std::string>();
        QuadrotorSim_SO3::Quadrotor quadrotor(simulator_config.string(), visualize);
        State simulator_state;
        simulator_state.p = state.pose.translation();
        simulator_state.rot = state.pose.rotation();
        simulator_state.v = state.velocity;
        simulator_state.body_rate = state.body_rate;
        quadrotor.setState(simulator_state);

        const auto initial_pose_noise = gtsam::noiseModel::Isotropic::Sigma(
            6, config["initial_pose_sigma"].as<double>());
        const auto initial_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
            3, config["initial_velocity_sigma"].as<double>());
        gtsam::Vector9 tracking_sigmas;
        tracking_sigmas <<
            gtsam::Vector3::Constant(config["terminal_position_sigma"].as<double>()),
            gtsam::Vector3::Constant(config["terminal_rotation_sigma"].as<double>()),
            gtsam::Vector3::Constant(config["terminal_velocity_sigma"].as<double>());
        const auto tracking_noise = gtsam::noiseModel::Diagonal::Sigmas(tracking_sigmas);
        const gtsam::Vector6 control_sigmas = controlSigmas(
            config["acceleration_regularization_sigma"].as<double>(),
            config["angular_speed_regularization_sigma"].as<double>());
        const auto control_noise = gtsam::noiseModel::Diagonal::Sigmas(control_sigmas);
        const gtsam::Matrix9 tracking_information =
            tracking_sigmas.cwiseInverse().cwiseAbs2().asDiagonal();
        const gtsam::Matrix6 control_information =
            control_sigmas.cwiseInverse().cwiseAbs2().asDiagonal();
        const UAVFactor::LqrTerminalWeight terminal_weight =
            UAVFactor::computeAccelerationGyroTerminalWeight(
                dt, tracking_information, control_information);
        const auto terminal_noise =
            gtsam::noiseModel::Gaussian::Information(terminal_weight.information);
        std::cout << "terminal_weight iterations=" << terminal_weight.iterations
                  << " riccati_residual=" << terminal_weight.riccati_residual
                  << " closed_loop_spectral_radius="
                  << terminal_weight.closed_loop_spectral_radius
                  << "\nterminal_information=\n"
                  << terminal_weight.information << '\n';
        const auto smoothness_noise = gtsam::noiseModel::Isotropic::Sigma(
            6, config["control_smoothness_sigma"].as<double>());

        gtsam::LevenbergMarquardtParams optimizer_params;
        optimizer_params.maxIterations = config["optimizer_iterations"].as<std::size_t>();
        optimizer_params.absoluteErrorTol = 1.0e-9;
        optimizer_params.relativeErrorTol = 1.0e-9;
        optimizer_params.verbosity = gtsam::NonlinearOptimizerParams::SILENT;

        gtsam::KeyVector control_keys;
        std::vector<gtsam::Vector6> warm_start(horizon, gtsam::Vector6::Zero());
        for (std::size_t k = 0; k < horizon; ++k) control_keys.push_back(U(k));

        double squared_position_error_sum = 0.0;
        double squared_rotation_error_sum = 0.0;
        std::vector<State> last_predicted_trajectory, last_reference_trajectory;
        gtsam::Vector3 last_position_error = gtsam::Vector3::Zero();
        gtsam::Vector3 last_attitude_error = gtsam::Vector3::Zero();
        float last_solve_cost = 0.0F;
        TimingStatistics solve_time_stats, cpu_time_stats, cpu_utilization_stats, cycle_time_stats;
        std::size_t deadline_misses = 0;

        for (std::size_t iteration = 0; iteration < mpc_iterations; ++iteration) {
            const auto frame_start = std::chrono::steady_clock::now();
            const double time = iteration * dt;
            gtsam::NonlinearFactorGraph graph;
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), state.pose, initial_pose_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), state.velocity,
                                                          initial_velocity_noise));

            // Factor j compares the fixed trajectory set point directly with
            // f(x_0, u_0, ..., u_{j-1}); no X(j) or V(j) variables are introduced.
            gtsam::KeyVector control_prefix;
            control_prefix.reserve(horizon);
            for (std::size_t j = 1; j <= horizon; ++j) {
                control_prefix.push_back(control_keys[j - 1]);
                const UAVFactor::DynamicsState set_point = circleReference(
                    circle_center, circle_radius, circle_angular_speed, time + j * dt,
                    face_velocity);
                graph.add(UAVFactor::TerminalAccelerationGyroMeasurementFactor(
                    X(0), V(0), control_prefix, set_point.pose, set_point.velocity, dt,
                    j == horizon ? terminal_noise : tracking_noise));
            }
            for (std::size_t k = 0; k < horizon; ++k) {
                graph.add(gtsam::PriorFactor<gtsam::Vector6>(U(k), gtsam::Vector6::Zero(),
                                                              control_noise));
                if (k > 0)
                    graph.add(gtsam::BetweenFactor<gtsam::Vector6>(
                        U(k - 1), U(k), gtsam::Vector6::Zero(), smoothness_noise));
            }

            gtsam::Values initial_values;
            initial_values.insert(X(0), state.pose);
            initial_values.insert(V(0), state.velocity);
            for (std::size_t k = 0; k < horizon; ++k)
                initial_values.insert(U(k), warm_start[k]);

            const double initial_cost = graph.error(initial_values);
            const auto solve_start = std::chrono::steady_clock::now();
            const std::clock_t cpu_start = std::clock();
            gtsam::Values result =
                gtsam::LevenbergMarquardtOptimizer(graph, initial_values, optimizer_params).optimize();
            result = enforceControlBounds(graph, result, control_keys, control_min, control_max,
                                          hard_control_constraint_iterations);
            const double control_violation =
                maximumControlViolation(result, control_keys, control_min, control_max);
            if (control_violation > 1.0e-8)
                throw std::runtime_error("Hard control bounds violated by " +
                                         std::to_string(control_violation));
            const double solve_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - solve_start).count();
            const double cpu_time_ms = 1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC;
            const double cpu_percent =
                solve_time_ms > 0.0 ? 100.0 * cpu_time_ms / solve_time_ms : 0.0;
            const double optimized_cost = graph.error(result);
            for (std::size_t k = 0; k < horizon; ++k)
                warm_start[k] = result.at<gtsam::Vector6>(U(k));

            const gtsam::Vector6 applied_control = warm_start.front();
            const gtsam::Vector3 acceleration = applied_control.head<3>();
            const gtsam::Vector3 angular_speed = applied_control.tail<3>();
            const UAVFactor::DynamicsState current_reference = circleReference(
                circle_center, circle_radius, circle_angular_speed, time + dt, face_velocity);

            // Convert the kinematic MPC command to the simulator's thrust/torque input.
            const double mass = quadrotor.getMass();
            const gtsam::Vector3 desired_force =
                mass * (acceleration + gtsam::Vector3(0.0, 0.0, quadrotor.getGravity()));
            const gtsam::Rot3 desired_rotation =
                thrustAlignedRotation(desired_force, current_reference.pose.rotation().yaw());
            const Eigen::Matrix3d inertia = quadrotor.getInertia();
            const std::size_t simulator_substeps =
                config["simulator_substeps"].as<std::size_t>();
            const double simulator_dt = dt / simulator_substeps;
            const double maximum_torque = config["maximum_torque"].as<double>();
            for (std::size_t substep = 0; substep < simulator_substeps; ++substep) {
                const gtsam::Vector3 attitude_error = gtsam::Rot3::Logmap(
                    simulator_state.rot.between(desired_rotation));
                const gtsam::Vector3 desired_body_rate =
                    angular_speed + config["attitude_gain"].as<double>() * attitude_error;
                gtsam::Vector3 torque =
                    inertia * (config["body_rate_gain"].as<double>() *
                               (desired_body_rate - simulator_state.body_rate)) +
                    simulator_state.body_rate.cross(inertia * simulator_state.body_rate);
                torque = torque.cwiseMax(gtsam::Vector3::Constant(-maximum_torque))
                             .cwiseMin(gtsam::Vector3::Constant(maximum_torque));
                gtsam::Vector4 thrust_torque;
                thrust_torque << std::max(0.0, desired_force.norm()), torque;
                quadrotor.stepODE(simulator_dt, thrust_torque);
                simulator_state = quadrotor.getState();
            }
            state.pose = gtsam::Pose3(simulator_state.rot, simulator_state.p);
            state.velocity = simulator_state.v;
            state.body_rate = simulator_state.body_rate;
            const double position_error =
                (state.pose.translation() - current_reference.pose.translation()).norm();
            const double rotation_error =
                rotationError(state.pose.rotation(), current_reference.pose.rotation());
            squared_position_error_sum += position_error * position_error;
            squared_rotation_error_sum += rotation_error * rotation_error;
            if (warm_start.size() > 1) {
                std::rotate(warm_start.begin(), warm_start.begin() + 1, warm_start.end());
                warm_start.back() = warm_start[warm_start.size() - 2];
            }

            std::cout << "iteration=" << iteration << " cost=" << initial_cost << "->"
                      << optimized_cost << " position_error="
                      << position_error << " rotation_error=" << rotation_error
                      << " control_bound_violation=" << control_violation
                      << " solve_ms=" << solve_time_ms << " cpu=" << cpu_percent << "%\n";

            if (iteration >= statistics_warmup) {
                solve_time_stats.add(solve_time_ms);
                cpu_time_stats.add(cpu_time_ms);
                cpu_utilization_stats.add(cpu_percent);
            }
            quadrotor.setPerformanceStats(solve_time_ms, solve_time_stats.mean(),
                                           solve_time_stats.percentile(0.95), cpu_percent,
                                           deadline_misses);

            if (visualize) {
                last_predicted_trajectory.clear();
                last_reference_trajectory.clear();
                UAVFactor::DynamicsState predicted = state;
                for (std::size_t k = 0; k < warm_start.size(); ++k) {
                    const auto& control = warm_start[k];
                    const gtsam::Vector3 a = control.head<3>();
                    predicted.pose = gtsam::Pose3(
                        predicted.pose.rotation() * gtsam::Rot3::Expmap(control.tail<3>() * dt),
                        predicted.pose.translation() + predicted.velocity * dt + 0.5 * a * dt * dt);
                    predicted.velocity += a * dt;
                    State point;
                    point.p = predicted.pose.translation();
                    point.rot = predicted.pose.rotation();
                    point.v = predicted.velocity;
                    last_predicted_trajectory.push_back(point);
                    const UAVFactor::DynamicsState reference = circleReference(
                        circle_center, circle_radius, circle_angular_speed,
                        time + (k + 2) * dt, face_velocity);
                    State reference_point;
                    reference_point.p = reference.pose.translation();
                    reference_point.rot = reference.pose.rotation();
                    reference_point.v = reference.velocity;
                    last_reference_trajectory.push_back(reference_point);
                }
                last_position_error =
                    state.pose.translation() - current_reference.pose.translation();
                last_attitude_error = gtsam::Rot3::Logmap(
                    current_reference.pose.rotation().between(state.pose.rotation()));
                last_solve_cost = static_cast<float>(optimized_cost);
                if (!quadrotor.renderHistoryOpt(
                        last_predicted_trajectory, last_position_error, boost::none, boost::none,
                        last_attitude_error, last_reference_trajectory, last_solve_cost))
                    break;
            }
            const double cycle_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - frame_start).count();
            if (iteration >= statistics_warmup) {
                cycle_time_stats.add(cycle_time_ms);
                if (cycle_time_ms > dt * 1000.0) ++deadline_misses;
            }
            if (statistics_print_interval > 0 &&
                (iteration + 1) % statistics_print_interval == 0) {
                std::cout << "timing samples=" << solve_time_stats.samples.size()
                          << " solve_mean_ms=" << solve_time_stats.mean()
                          << " solve_p95_ms=" << solve_time_stats.percentile(0.95)
                          << " cycle_mean_ms=" << cycle_time_stats.mean()
                          << " cpu_mean=" << cpu_utilization_stats.mean() << "%"
                          << " deadline_misses=" << deadline_misses << '\n';
            }
            if (visualize && real_time_playback) {
                std::this_thread::sleep_until(frame_start +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(dt)));
            }
        }

        const double rms_position_error =
            std::sqrt(squared_position_error_sum / mpc_iterations);
        const double rms_rotation_error =
            std::sqrt(squared_rotation_error_sum / mpc_iterations);
        std::cout << "final_position=" << state.pose.translation().transpose()
                  << " final_velocity=" << state.velocity.transpose()
                  << " rms_position_error=" << rms_position_error
                  << " rms_rotation_error=" << rms_rotation_error << '\n';
        std::cout << "timing_summary samples=" << solve_time_stats.samples.size()
                  << " solve_ms[min/mean/std/p95/max]=" << solve_time_stats.minimum() << "/"
                  << solve_time_stats.mean() << "/" << solve_time_stats.standardDeviation() << "/"
                  << solve_time_stats.percentile(0.95) << "/" << solve_time_stats.maximum()
                  << " cpu_time_mean_ms=" << cpu_time_stats.mean()
                  << " cpu_utilization_mean=" << cpu_utilization_stats.mean() << "%"
                  << " cycle_mean_ms=" << cycle_time_stats.mean()
                  << " deadline_misses=" << deadline_misses << '\n';
        if (visualize && !last_predicted_trajectory.empty()) {
            while (quadrotor.renderHistoryOpt(
                last_predicted_trajectory, last_position_error, boost::none, boost::none,
                last_attitude_error, last_reference_trajectory, last_solve_cost)) {
            }
        }
        return rms_position_error <= config["success_rms_position_tolerance"].as<double>() &&
                       rms_rotation_error <= config["success_rms_rotation_tolerance"].as<double>()
                   ? 0
                   : 2;
    } catch (const std::exception& exception) {
        std::cerr << "terminal_acceleration_gyro_setpoint_mpc: " << exception.what() << '\n';
        return 1;
    }
}
