#include <algorithm>
#include <chrono>
#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/cbf_factor.h>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <ipn_mpc/trajectory/trajectory_generator.h>
#include <random>
#include <yaml-cpp/yaml.h>

using namespace gtsam;
using namespace QuadrotorSim_SO3;
using namespace std;
using namespace Trajectory;
using namespace UAVFactor;

using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

namespace {

constexpr double kMinimumDistance = 1.0e-6;

} // namespace

int main(int argc, char** argv) {
    const std::string factor_graph_config_path =
        argc > 1 ? argv[1] : "../config/factor_graph_thrust_gyro_cbf.yaml";
    const std::string quadrotor_config_path =
        argc > 2 ? argv[2] : "../config/quadrotor_thrust_gyro_cbf.yaml";
    IPN_LOG_INFO << "factor_graph_config=" << factor_graph_config_path
                 << " quadrotor_config=" << quadrotor_config_path;

    const YAML::Node graph_config = YAML::LoadFile(factor_graph_config_path);
    const double initial_position_sigma = graph_config["pri_vicon_cov"].as<double>();
    const double initial_velocity_sigma = graph_config["pri_vicon_vel_cov"].as<double>();
    const Vector3 stage_position_sigmas(graph_config["control_p_cov_x"].as<double>(),
                                        graph_config["control_p_cov_y"].as<double>(),
                                        graph_config["control_p_cov_z"].as<double>());
    const Vector3 terminal_position_sigmas(graph_config["control_p_final_cov_x"].as<double>(),
                                           graph_config["control_p_final_cov_y"].as<double>(),
                                           graph_config["control_p_final_cov_z"].as<double>());
    const Vector3 rotation_tracking_sigmas(graph_config["control_r1_cov"].as<double>(),
                                           graph_config["control_r2_cov"].as<double>(),
                                           graph_config["control_r3_cov"].as<double>());
    const double velocity_tracking_sigma = graph_config["control_v_cov"].as<double>();
    const double dynamics_sigma = graph_config["dynamic_t_cov"].as<double>();
    const size_t horizon_length = graph_config["opt_lens_traj"].as<size_t>();
    const Vector4 control_smoothness_sigmas(graph_config["input_jerk_t"].as<double>(),
                                            graph_config["input_jerk_m"].as<double>(),
                                            graph_config["input_jerk_m"].as<double>(),
                                            graph_config["input_jerk_m3"].as<double>());
    const size_t simulation_steps = graph_config["sim_steps"].as<size_t>();
    const std::string log_name = graph_config["log_name"].as<std::string>();
    const double obstacle_sigma = graph_config["point_obs_sigma"].as<double>();
    const double maximum_thrust = graph_config["clf_high"].as<double>();
    const double minimum_thrust = graph_config["clf_low"].as<double>();
    const double thrust_limit_threshold = graph_config["clf_thr"].as<double>();
    const double maximum_body_rate = graph_config["g_clf_high"].as<double>();
    const double minimum_body_rate = graph_config["g_clf_low"].as<double>();
    const double body_rate_limit_threshold = graph_config["g_clf_thr"].as<double>();
    const double control_limit_alpha = graph_config["clf_alpha"].as<double>();
    const size_t max_optimizer_iterations = graph_config["max_iters"].as<size_t>();
    const double cbf_distance_gain = graph_config["cbf_alpha"].as<double>();
    const double cbf_velocity_gain = graph_config["cbf_beta"].as<double>();

    const YAML::Node simulator_config = YAML::LoadFile(quadrotor_config_path);

    const double position_measurement_mean = simulator_config["pos_meas_mean"].as<double>();
    const double position_measurement_sigma = simulator_config["pos_meas_cov"].as<double>();
    const double velocity_measurement_sigma = simulator_config["vel_meas_cov"].as<double>();
    const double rotation_measurement_sigma = simulator_config["rot_meas_cov"].as<double>();
    const bool enable_recovery_test = simulator_config["test_recovery"].as<bool>();
    const double safety_margin = simulator_config["safe_d"].as<double>();
    const double vehicle_radius = simulator_config["uav_size"].as<double>() / 2.0;
    const Vector3 recovery_offset(simulator_config["move_x"].as<double>(),
                                  simulator_config["move_y"].as<double>(),
                                  simulator_config["move_z"].as<double>());
    const Vector3 drag_coefficients(simulator_config["drag_force_x"].as<double>(),
                                    simulator_config["drag_force_y"].as<double>(),
                                    simulator_config["drag_force_z"].as<double>());
    constexpr double kVehicleMass = 1.0;
    constexpr double kControlPeriod = 0.01;

    std::ofstream trajectory_log;
    std::string file_name = "../data/log/JPC_TGyro_";
    file_name.append(log_name);
    file_name.append("_log.txt");
    trajectory_log.open(file_name);

    Trajectory::back_and_forth_generator reference_generator(5.0, 2.0, 0.001);

    gtsam::LevenbergMarquardtParams optimizer_parameters;
    optimizer_parameters.absoluteErrorTol = 100;
    optimizer_parameters.relativeErrorTol = 1e-2;
    optimizer_parameters.maxIterations = max_optimizer_iterations;
    optimizer_parameters.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    optimizer_parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;

    auto control_smoothness_noise = noiseModel::Diagonal::Sigmas(control_smoothness_sigmas);
    auto dynamics_noise = noiseModel::Diagonal::Sigmas(
        (Vector(9) << Vector3::Constant(dynamics_sigma * 0.5 * kControlPeriod * kControlPeriod),
         Vector3::Constant(dynamics_sigma * kControlPeriod), Vector3::Constant(0.01))
            .finished());

    // Initial state noise
    auto initial_pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(rotation_measurement_sigma),
         Vector3::Constant(initial_position_sigma))
            .finished());
    auto initial_velocity_noise = noiseModel::Diagonal::Sigmas(
        Vector3::Constant(initial_velocity_sigma));
    auto obstacle_noise = noiseModel::Diagonal::Sigmas(Vector1(obstacle_sigma));

    auto tracking_velocity_noise =
        noiseModel::Diagonal::Sigmas(Vector3::Constant(velocity_tracking_sigma));
    Quadrotor quadrotor(quadrotor_config_path);
    State simulated_state;
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;

    std::default_random_engine meas_rx_gen;
    std::default_random_engine meas_ry_gen;
    std::default_random_engine meas_rz_gen;

    std::default_random_engine meas_vx_gen;
    std::default_random_engine meas_vy_gen;
    std::default_random_engine meas_vz_gen;

    std::normal_distribution<double> position_noise(position_measurement_mean,
                                                    position_measurement_sigma);
    std::normal_distribution<double> rotation_noise(0.0, rotation_measurement_sigma);
    std::normal_distribution<double> velocity_noise(0.0, velocity_measurement_sigma);

    gtsam::Vector3 vicon_measurement;
    std::vector<Obstacle> obstacles;

    gtsam::Matrix3 planar_projection;
    planar_projection << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    for (size_t simulation_step = 0; simulation_step < simulation_steps; ++simulation_step) {
        obstacles = quadrotor.getObstacles();
        const double current_time = simulation_step * kControlPeriod;
        std::vector<State> predicted_trajectory, reference_trajectory;
        State reference_state;

        if (simulation_step == 0) {
            State init_state;
            init_state.p = reference_generator.pos(0.0);
            init_state.rot = gtsam::Rot3::Expmap(reference_generator.theta(0.0));
            init_state.v = reference_generator.vel(0.0);
            init_state.body_rate = reference_generator.omega(0.0);

            quadrotor.setState(init_state);
            simulated_state = init_state;

            // Submit the first Pangolin frame before the optimizer starts so the window is never
            // left blank during optimization initialization.
            if (!quadrotor.renderHistoryTrj()) {
                return 0;
            }
        }

        if (simulation_step == 1000 && enable_recovery_test) {
            simulated_state.p += recovery_offset;
            quadrotor.setState(simulated_state);
        }

        NonlinearFactorGraph graph;
        Values initial_value;
        auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
        graph.emplace_shared<ControlLimitTGyroFactor>(
            U(0), clf_sigma, minimum_thrust, maximum_thrust, minimum_body_rate,
            maximum_body_rate, thrust_limit_threshold, body_rate_limit_threshold,
            control_limit_alpha);

        for (size_t idx = 0; idx < horizon_length; ++idx) {
            DynamicsFactorTGyro dynamics_factor(X(idx), V(idx), U(idx), X(idx + 1), V(idx + 1),
                                                kControlPeriod, kVehicleMass, drag_coefficients,
                                                dynamics_noise);
            graph.add(dynamics_factor);

            const double prediction_time = current_time + (idx + 1) * kControlPeriod;
            gtsam::Pose3 reference_pose(
                gtsam::Rot3::Expmap(reference_generator.theta(prediction_time)),
                reference_generator.pos(prediction_time));
            const gtsam::Vector3 reference_velocity = reference_generator.vel(prediction_time);

            reference_state.p = reference_pose.translation();
            reference_state.rot = reference_pose.rotation();
            reference_state.v = reference_velocity;
            reference_trajectory.push_back(reference_state);

            gtsam::Vector4 init_input(10, 0, 0, 0);
            initial_value.insert(U(idx), init_input);

            for (const Obstacle& obstacle : obstacles) {
                const gtsam::Vector3 predicted_obstacle_position =
                    obstacle.obs_pos + obstacle.obs_vel * (idx + 1) * kControlPeriod;
                if (obstacle.obs_type == ObsType::sphere) {
                    const double distance = (reference_pose.translation() - predicted_obstacle_position).norm();

                    if (distance < obstacle.obs_size + safety_margin + vehicle_radius) {
                        const double safe_distance = std::max(distance, kMinimumDistance);
                        const double scale = (obstacle.obs_size + safety_margin + vehicle_radius) / safe_distance;
                        reference_pose =
                            gtsam::Pose3(reference_pose.rotation(),
                                         (reference_pose.translation() - predicted_obstacle_position) * scale +
                                             predicted_obstacle_position);
                    }
                } else if (obstacle.obs_type == ObsType::cylinder) {
                    const double distance =
                        (planar_projection *
                         (reference_pose.translation() - predicted_obstacle_position))
                            .norm();

                    if (distance < obstacle.obs_size + safety_margin + vehicle_radius) {
                        const double safe_distance = std::max(distance, kMinimumDistance);
                        const double scale = (obstacle.obs_size + safety_margin + vehicle_radius) / safe_distance;

                        gtsam::Point3 current_pos = reference_pose.translation();
                        gtsam::Point3 obs_pos = predicted_obstacle_position;

                        gtsam::Point3 new_xy_pos(
                            (current_pos.x() - obs_pos.x()) * scale + obs_pos.x(),
                            (current_pos.y() - obs_pos.y()) * scale + obs_pos.y(), current_pos.z());

                        reference_pose = gtsam::Pose3(reference_pose.rotation(), new_xy_pos);
                    }
                }
            }

            initial_value.insert(X(idx + 1), reference_pose);
            initial_value.insert(V(idx + 1), reference_velocity);

            if (idx != 0) {
                BetForceMoments bet_FM_factor(U(idx - 1), U(idx), control_smoothness_noise);
                graph.add(bet_FM_factor);
            }
            // initial_value.insert(U(idx), init_input); //
            // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            if (idx == horizon_length - 1) {
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << rotation_tracking_sigmas, terminal_position_sigmas).finished());
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), reference_pose, ref_predict_pose_noise));
                graph.add(
                    gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), reference_velocity, tracking_velocity_noise));
            } else {
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << rotation_tracking_sigmas, stage_position_sigmas).finished());
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), reference_pose, ref_predict_pose_noise));
                graph.add(
                    gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), reference_velocity, tracking_velocity_noise));
            }

            for (const Obstacle& obstacle : obstacles) {
                const gtsam::Vector3 predicted_obstacle_position =
                    obstacle.obs_pos + obstacle.obs_vel * (idx + 1) * kControlPeriod;
                if (obstacle.obs_type == ObsType::sphere) {
                    // graph.add(PointObsFactor(X(idx+1), obstacle, obs1_radius + safety_margin,
                    // obstacle_noise));
                    if (idx == horizon_length - 1) {
                        graph.add(CBFPdFactor(X(idx + 1), V(idx + 1), predicted_obstacle_position,
                                              obstacle.obs_size + safety_margin + vehicle_radius,
                                              cbf_distance_gain,
                                              obstacle_noise));
                    } else {
                        graph.add(VeCBFPdFactor1(X(idx + 1), V(idx + 1), U(idx), predicted_obstacle_position,
                                                 obstacle.obs_vel, obstacle.obs_size + safety_margin + vehicle_radius,
                                                 cbf_distance_gain, cbf_velocity_gain, obstacle_noise));
                    }
                } else if (obstacle.obs_type == ObsType::cylinder) {
                    if (idx == horizon_length - 1) {
                        graph.add(CBFPdFactorCylinder(X(idx + 1), V(idx + 1), predicted_obstacle_position,
                                                      obstacle.obs_size + safety_margin + vehicle_radius,
                                                      cbf_distance_gain, obstacle_noise));
                    } else {
                        graph.add(VeCBFPdFactorCylinder1(X(idx + 1), V(idx + 1), U(idx),
                                                         predicted_obstacle_position, obstacle.obs_vel,
                                                         obstacle.obs_size + safety_margin + vehicle_radius,
                                                         cbf_distance_gain, cbf_velocity_gain,
                                                         obstacle_noise));
                    }
                }
            }

            if (idx == 0) {
                gtsam::Vector3 pos_noise =
                    gtsam::Vector3(position_noise(meas_x_gen), position_noise(meas_y_gen),
                                   position_noise(meas_z_gen));
                gtsam::Vector3 vel_noise_add =
                    gtsam::Vector3(velocity_noise(meas_vx_gen), velocity_noise(meas_vy_gen),
                                   velocity_noise(meas_vz_gen));
                gtsam::Vector3 rot_noise_add =
                    gtsam::Vector3(rotation_noise(meas_rx_gen), rotation_noise(meas_ry_gen),
                                   rotation_noise(meas_rz_gen));

                vicon_measurement = simulated_state.p + pos_noise;
                gtsam::Vector3 vel_add = simulated_state.v + vel_noise_add;
                gtsam::Vector3 rot_add = gtsam::Rot3::Logmap(simulated_state.rot) + rot_noise_add;

                graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    X(idx), gtsam::Pose3(gtsam::Rot3::Expmap(rot_add), vicon_measurement),
                    initial_pose_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), vel_add, initial_velocity_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), simulated_state.body_rate,
                // omega_noise));

                initial_value.insert(X(idx), gtsam::Pose3(simulated_state.rot, vicon_measurement));
                initial_value.insert(V(idx), vel_add);
                // initial_value.insert(S(idx), simulated_state.body_rate);
            }
        }

        LevenbergMarquardtOptimizer optimizer(graph, initial_value, optimizer_parameters);
        IPN_LOG_DEBUG << "Optimizer: starting solve";
        const auto solve_start = std::chrono::steady_clock::now();
        Values result = optimizer.optimize();
        float solve_time_seconds =
            std::chrono::duration<float>(std::chrono::steady_clock::now() - solve_start).count();
        IPN_LOG_DEBUG << "Optimizer: solve_time_ms=" << solve_time_seconds * 1000.0;

        for (size_t step = 0; step <= horizon_length; ++step) {
            State predicted_state;
            predicted_state.p = result.at<Pose3>(X(step)).translation();
            predicted_trajectory.push_back(predicted_state);
        }

        const Vector4 control_input = result.at<Vector4>(U(0));

        // IPN_LOG_DEBUG << "Control control_input: value=" << control_input.transpose();

        /* Simulator */
        State simulator_state = quadrotor.getState();
        gtsam::Vector3 drag_force = simulator_state.rot.matrix() * Eigen::Matrix3d(drag_coefficients.asDiagonal()) *
                                    simulator_state.rot.matrix().transpose() * simulator_state.v;
        IPN_LOG_DEBUG << "Aerodynamic drag force [N]: " << drag_force.transpose();

        float g_ = 9.81f;
        std::normal_distribution<double> thrust_noise(0, 0.2);
        std::normal_distribution<double> gyro_noise(0, 0.01);
        std::default_random_engine generator_;
        double at_noise = thrust_noise(generator_);
        double wx_noise = gyro_noise(generator_);
        double wy_noise = gyro_noise(generator_);
        double wz_noise = gyro_noise(generator_);

        gtsam::Vector3 v_dot =
            -gtsam::Vector3(0, 0, g_) +
            simulator_state.rot.rotate(
                gtsam::Vector3(0, 0, (control_input[0] + at_noise) / kVehicleMass)) +
            drag_force / kVehicleMass;
        gtsam::Vector3 p_dot = simulator_state.v;
        simulator_state.p += p_dot * kControlPeriod;
        simulator_state.v += v_dot * kControlPeriod;
        gtsam::Vector3 body_rate = gtsam::Vector3(control_input[1], control_input[2], control_input[3]) +
                                   gtsam::Vector3(wx_noise, wy_noise, wz_noise);
        simulator_state.rot =
            simulator_state.rot * gtsam::Rot3::Expmap(body_rate * kControlPeriod);

        quadrotor.setState(simulator_state);

        simulated_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(simulated_state.rot, simulated_state.p);

        gtsam::Vector3 tar_position = reference_generator.pos(current_time + kControlPeriod);
        gtsam::Vector3 tar_theta = reference_generator.theta(current_time + kControlPeriod);
        gtsam::Rot3 tar_rotation = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel = reference_generator.vel(current_time + kControlPeriod);
        gtsam::Vector3 tar_omega = reference_generator.omega(current_time + kControlPeriod);
        gtsam::Vector4 ref_input = reference_generator.inputfm(current_time);

        gtsam::Vector3 pos_err = simulated_state.p - tar_position;
        gtsam::Vector3 rot_err = tar_rotation.rpy() - simulated_state.rot.rpy();

        if (!quadrotor.renderHistoryOpt(predicted_trajectory, pos_err, boost::none, vicon_measurement, rot_err,
                                        reference_trajectory, solve_time_seconds)) {
            break;
        }

        if (obstacles.size() == 1) {
            trajectory_log << predicted_pose.translation().x() << " " << predicted_pose.translation().y()
                    << " " << predicted_pose.translation().z() << " "
                    << simulated_state.rot.rpy().x() << " " << simulated_state.rot.rpy().y() << " "
                    << simulated_state.rot.rpy().z() << " " << simulated_state.v.x() << " "
                    << simulated_state.v.y() << " " << simulated_state.v.z() << " "
                    << simulated_state.body_rate.x() << " " << simulated_state.body_rate.y() << " "
                    << simulated_state.body_rate.z() << " " << control_input[0] << " " << control_input[1] << " "
                    << control_input[2] << " " << control_input[3] << " " << tar_position.x() << " "
                    << tar_position.y() << " " << tar_position.z() << " " << tar_rotation.rpy().x()
                    << " " << tar_rotation.rpy().y() << " " << tar_rotation.rpy().z() << " "
                    << tar_vel.x() << " " << tar_vel.y() << " " << tar_vel.z() << " "
                    << tar_omega.x() << " " << tar_omega.y() << " " << tar_omega.z() << " "
                    << ref_input[0] << " " << ref_input[1] << " " << ref_input[2] << " "
                    << ref_input[3] << " " << solve_time_seconds << " " << obstacles[0].obs_pos.x() << " "
                    << obstacles[0].obs_pos.y() << " " << obstacles[0].obs_pos.z() << " "
                    << obstacles[0].obs_vel.x() << " " << obstacles[0].obs_vel.y() << " "
                    << obstacles[0].obs_vel.z() << " ";
        } else {
            trajectory_log << predicted_pose.translation().x() << " " << predicted_pose.translation().y()
                    << " " << predicted_pose.translation().z() << " "
                    << simulated_state.rot.rpy().x() << " " << simulated_state.rot.rpy().y() << " "
                    << simulated_state.rot.rpy().z() << " " << simulated_state.v.x() << " "
                    << simulated_state.v.y() << " " << simulated_state.v.z() << " "
                    << simulated_state.body_rate.x() << " " << simulated_state.body_rate.y() << " "
                    << simulated_state.body_rate.z() << " " << control_input[0] << " " << control_input[1] << " "
                    << control_input[2] << " " << control_input[3] << " " << tar_position.x() << " "
                    << tar_position.y() << " " << tar_position.z() << " " << tar_rotation.rpy().x()
                    << " " << tar_rotation.rpy().y() << " " << tar_rotation.rpy().z() << " "
                    << tar_vel.x() << " " << tar_vel.y() << " " << tar_vel.z() << " "
                    << tar_omega.x() << " " << tar_omega.y() << " " << tar_omega.z() << " "
                    << ref_input[0] << " " << ref_input[1] << " " << ref_input[2] << " "
                    << ref_input[3] << " " << solve_time_seconds << " ";
        }
    }

    while (quadrotor.renderHistoryTrj()) {
    }

    return 0;
}
