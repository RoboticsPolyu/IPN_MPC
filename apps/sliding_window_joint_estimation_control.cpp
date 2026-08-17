#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <ipn_mpc/simulation/landmarks.h>
#include <ipn_mpc/simulation/lidar.h>
#include <ipn_mpc/trajectory/trajectory_generator.h>
#include <yaml-cpp/yaml.h>

using namespace Env_Sim;
using namespace gtsam;
using namespace QuadrotorSim_SO3;
using namespace Sensors_Sim;
using namespace std;
using namespace Trajectory;
using namespace UAVFactor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(int argc, char** argv) {

    const std::string factor_graph_config_path =
        argc > 1 ? argv[1] : "../config/factor_graph.yaml";
    const std::string quadrotor_config_path =
        argc > 2 ? argv[2] : "../config/quadrotor.yaml";
    const std::string simulator_config_path =
        argc > 3 ? argv[3] : "../config/quadrotor_thrust_gyro.yaml";
    IPN_LOG_INFO << "factor_graph_config=" << factor_graph_config_path
                 << " quadrotor_config=" << quadrotor_config_path
                 << " simulator_config=" << simulator_config_path;

    // Configuration file
    YAML::Node FGO_config = YAML::LoadFile(factor_graph_config_path);
    double pri_vicon_cov = FGO_config["pri_vicon_cov"].as<double>();
    double pri_vicon_vel_cov = FGO_config["pri_vicon_vel_cov"].as<double>();
    double control_p_cov_x = FGO_config["control_p_cov_x"].as<double>();
    double control_p_cov_y = FGO_config["control_p_cov_y"].as<double>();
    double control_p_cov_z = FGO_config["control_p_cov_z"].as<double>();
    double control_p_final_cov_x = FGO_config["control_p_final_cov_x"].as<double>();
    double control_p_final_cov_y = FGO_config["control_p_final_cov_y"].as<double>();
    double control_p_final_cov_z = FGO_config["control_p_final_cov_z"].as<double>();

    double control_o_cov = FGO_config["control_o_cov"].as<double>();

    double control_v_cov = FGO_config["control_v_cov"].as<double>();
    double dynamic_p_cov = FGO_config["dynamic_p_cov"].as<double>();
    double control_r_cov = FGO_config["control_r_cov"].as<double>();
    uint16_t opt_lens_traj = FGO_config["opt_lens_traj"].as<uint16_t>();

    double prior_u_f_cov = FGO_config["prior_u_f_cov"].as<double>();
    double prior_u_m1_cov = FGO_config["prior_u_m1_cov"].as<double>();
    double prior_u_m2_cov = FGO_config["prior_u_m2_cov"].as<double>();
    double prior_u_m3_cov = FGO_config["prior_u_m3_cov"].as<double>();

    double input_jerk_t = FGO_config["input_jerk_t"].as<double>();
    double input_jerk_m = FGO_config["input_jerk_m"].as<double>();

    uint64_t sim_steps = FGO_config["sim_steps"].as<uint64_t>();

    std::string log_name = FGO_config["log_name"].as<std::string>();

    uint16_t window_size = FGO_config["window_size"].as<uint16_t>();

    YAML::Node quad_config = YAML::LoadFile(quadrotor_config_path);

    double radius = quad_config["radius"].as<double>();
    double linear_vel = quad_config["linear_vel"].as<double>();
    double pos_meas_cov = quad_config["pos_meas_cov"].as<double>();
    double vel_meas_cov = quad_config["vel_meas_cov"].as<double>();
    double pos_meas_mean = quad_config["pos_meas_mean"].as<double>();
    bool test_recovery = quad_config["test_recovery"].as<bool>();

    double map_x = quad_config["map_x"].as<double>();
    double map_y = quad_config["map_y"].as<double>();
    double map_z = quad_config["map_z"].as<double>();

    double map_center_x = quad_config["map_center_x"].as<double>();
    double map_center_y = quad_config["map_center_y"].as<double>();
    double map_center_z = quad_config["map_center_z"].as<double>();
    double lidar_range = quad_config["lidar_range"].as<double>();
    double lidar_range_min = quad_config["lidar_range_min"].as<double>();
    double landmarks_size = quad_config["landmarks_size"].as<uint32_t>();

    double move_x = quad_config["move_x"].as<double>();
    double move_y = quad_config["move_y"].as<double>();
    double move_z = quad_config["move_z"].as<double>();

    std::ofstream JEC_log;
    std::string file_name = "../data/JEC_";
    file_name.append(log_name);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    double dt = 0.001f;
    circle_generator circle_generator(radius, linear_vel, dt);

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations = 500;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;

    auto input_jerk = noiseModel::Diagonal::Sigmas(
        Vector4(input_jerk_t, input_jerk_m, input_jerk_m, input_jerk_m));
    auto input_noise = noiseModel::Diagonal::Sigmas(
        Vector4(prior_u_f_cov, prior_u_m1_cov, prior_u_m2_cov, prior_u_m3_cov));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas(
        (Vector(12) << Vector3::Constant(dynamic_p_cov), Vector3::Constant(0.0005),
         Vector3::Constant(0.0005), Vector3::Constant(0.0005))
            .finished());

    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.001), Vector3::Constant(pri_vicon_cov)).finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(
        Vector3(pri_vicon_vel_cov, pri_vicon_vel_cov, pri_vicon_vel_cov));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    auto ref_predict_vel_noise =
        noiseModel::Diagonal::Sigmas(Vector3(control_v_cov, control_v_cov, control_v_cov));
    auto ref_predict_omega_noise =
        noiseModel::Diagonal::Sigmas(Vector3(control_o_cov, control_o_cov, control_o_cov));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor(simulator_config_path);
    State last_state, predicted_state;
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;

    std::default_random_engine meas_vx_gen;
    std::default_random_engine meas_vy_gen;
    std::default_random_engine meas_vz_gen;

    std::default_random_engine meas_lidar_gen;

    std::normal_distribution<double> position_noise(pos_meas_mean, pos_meas_cov);
    std::normal_distribution<double> velocity_noise(0, vel_meas_cov);
    std::normal_distribution<double> lidar_noise(0, 0.003);

    Features landmarkk;
    Landmarks env(map_x, map_y, map_z, map_center_x, map_center_y, map_center_z, landmarks_size);
    Lidar<Landmarks> lidar(lidar_range, lidar_range_min);
    gtsam::Vector3 vicon_measurement;

    std::vector<State> measurements;
    std::vector<gtsam::Pose3> lidar_measures;
    std::vector<State> opt_trj, state_trj;

    for (int traj_idx = 0; traj_idx < sim_steps; traj_idx++) {
        double t0 = traj_idx * dt;

        if (traj_idx == 0) {
            predicted_state.p = circle_generator.pos(t0);
            predicted_state.rot = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            predicted_state.v = circle_generator.vel(t0);
            predicted_state.body_rate = circle_generator.omega(t0);
            predicted_state.thrust_torque = circle_generator.inputfm(t0);
            quadrotor.setState(predicted_state);
        }

        if (traj_idx == 50 && test_recovery) {
            predicted_state.p[0] = predicted_state.p[0] + move_x;
            predicted_state.p[1] = predicted_state.p[1] + move_y;
            predicted_state.p[2] = predicted_state.p[2] + move_z;
            quadrotor.setState(predicted_state);
        }

        NonlinearFactorGraph graph;
        Values initial_value;
        graph.empty();

        gtsam::Vector4 input_bak;

        gtsam::Vector3 pos_noise = gtsam::Vector3(
            position_noise(meas_x_gen), position_noise(meas_y_gen), position_noise(meas_z_gen));
        gtsam::Vector3 vel_noise_add = gtsam::Vector3(
            velocity_noise(meas_vx_gen), velocity_noise(meas_vy_gen), velocity_noise(meas_vz_gen));
        gtsam::Vector6 lidar_noise_add;
        lidar_noise_add << lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen),
            lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen),
            lidar_noise(meas_lidar_gen);

        vicon_measurement = predicted_state.p + pos_noise;
        gtsam::Vector3 vel_add = predicted_state.v + vel_noise_add;

        // vicon_measurement = predicted_state.p;
        // gtsam::Vector3 vel_add = predicted_state.v;

        State m_state;
        m_state.p = vicon_measurement;
        m_state.rot = predicted_state.rot;
        m_state.v = vel_add;
        m_state.body_rate = predicted_state.body_rate;

        measurements.push_back(m_state);

        if (traj_idx > 0) {
            gtsam::Pose3 lidar_pR(last_state.rot.inverse() * predicted_state.rot,
                                  last_state.rot.unrotate(predicted_state.p - last_state.p));
            lidar_measures.push_back(lidar_pR);
        }

        if (measurements.size() > window_size) {
            measurements.erase(measurements.begin());
        }
        if (lidar_measures.size() > window_size - 1) {
            lidar_measures.erase(lidar_measures.begin());
        }

        IPN_LOG_DEBUG << "Trajectory: index=" << traj_idx;

        gtsam::Vector4 input;
        if (traj_idx >= window_size - 1) {
            for (int idx = 0; idx < opt_lens_traj + window_size; idx++) {
                if (idx >= window_size - 1) {
                    DynamicsFactorTm dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1),
                                                     V(idx + 1), S(idx + 1), dt, dynamics_noise);
                    graph.add(dynamics_factor);

                    int future_idx = (idx - window_size + 2);

                    gtsam::Pose3 pose_idx(
                        gtsam::Rot3::Expmap(circle_generator.theta(t0 + future_idx * dt)),
                        circle_generator.pos(t0 + future_idx * dt));

                    gtsam::Vector3 vel_idx = circle_generator.vel(t0 + future_idx * dt);
                    gtsam::Vector3 omega_idx = circle_generator.omega(t0 + future_idx * dt);

                    initial_value.insert(X(idx + 1), pose_idx);
                    initial_value.insert(V(idx + 1), vel_idx);
                    initial_value.insert(S(idx + 1), omega_idx);

                    gtsam::Vector4 init_input = circle_generator.inputfm(t0 + future_idx * dt);

                    if (idx != window_size - 1) {
                        BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                        graph.add(bet_FM_factor);
                    }
                    initial_value.insert(U(idx), init_input);
                    // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input,
                    // input_noise));

                    if (idx == opt_lens_traj + window_size - 1) {
                        gtsam::Vector3 final_position_ref(
                            control_p_final_cov_x, control_p_final_cov_y, control_p_final_cov_z);
                        auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                            (Vector(6) << Vector3::Constant(control_r_cov), final_position_ref)
                                .finished());
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx,
                                                                   ref_predict_pose_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,
                        // ref_predict_vel_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx,
                        // ref_predict_omega_noise));
                    } else {
                        gtsam::Vector3 _position_ref(control_p_cov_x, control_p_cov_y,
                                                     control_p_cov_z);
                        auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                            (Vector(6) << Vector3::Constant(control_r_cov), _position_ref)
                                .finished());
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx,
                                                                   ref_predict_pose_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,
                        // ref_predict_vel_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx,
                        // ref_predict_omega_noise));
                    }
                }

                if (idx < window_size) {
                    if (idx == window_size - 1) {
                        graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), measurements[idx].v,
                                                                     vel_noise));
                        graph.add(gtsam::PriorFactor<gtsam::Vector3>(
                            S(idx), measurements[idx].body_rate, omega_noise));
                        initial_value.insert(V(idx), measurements[idx].v);
                        initial_value.insert(S(idx), measurements[idx].body_rate);
                    }

                    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                        X(idx), gtsam::Pose3(measurements[idx].rot, measurements[idx].p),
                        vicon_noise));

                    if (idx > 0) {
                        auto bet_noise = noiseModel::Diagonal::Sigmas(
                            (Vector(6) << Vector3::Constant(0.003), Vector3::Constant(0.003))
                                .finished());

                        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            X(idx - 1), X(idx),
                            lidar_measures[idx].compose(
                                gtsam::Pose3(gtsam::Rot3::Expmap(lidar_noise_add.tail(3)),
                                             gtsam::Vector3(lidar_noise_add.head(3)))),
                            bet_noise));
                    }

                    initial_value.insert(X(idx),
                                         gtsam::Pose3(measurements[idx].rot, measurements[idx].p));
                }
            }

            IPN_LOG_DEBUG << "Optimizer: constructing";
            LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

            IPN_LOG_DEBUG << "Optimizer: starting solve";
            Values result = optimizer.optimize();

            gtsam::Pose3 i_pose;
            gtsam::Vector3 vel;
            gtsam::Vector3 omega;

            state_trj.clear();
            opt_trj.clear();

            for (uint32_t ikey = 0; ikey < opt_lens_traj + window_size; ikey++) {
                IPN_LOG_DEBUG << "Trajectory optimization: state_index=" << ikey;
                i_pose = result.at<Pose3>(X(ikey));
                IPN_LOG_DEBUG << "Optimized position [m]: " << i_pose.translation();
                double t_ref = t0 + (ikey - window_size + 1) * dt;

                gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t_ref)),
                                      circle_generator.pos(t_ref));
                IPN_LOG_DEBUG << "Reference position [m]: " << ref_pose.translation();

                IPN_LOG_DEBUG << "Optimized rotation logmap [rad]: "
                              << Rot3::Logmap(i_pose.rotation()).transpose();
                IPN_LOG_DEBUG << "Reference rotation logmap [rad]: "
                              << Rot3::Logmap(ref_pose.rotation()).transpose();

                // vel = result.at<Vector3>(V(ikey));
                // IPN_LOG_DEBUG << "Optimized velocity [m/s]: "
                //         << vel.transpose();
                // gtsam::Vector3 ref_vel = circle_generator.vel(t_ref);
                // //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)),
                // circle_generator.pos(ikey * dt)); IPN_LOG_DEBUG << "Reference velocity [m/s]: "
                //         << ref_vel.transpose();

                // omega = result.at<Vector3>(S(ikey));
                // IPN_LOG_DEBUG << "Optimized angular velocity [rad/s]: "
                //         << omega.transpose();
                // gtsam::Vector3 ref_omega = circle_generator.omega(t_ref);
                // //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)),
                // circle_generator.pos(ikey * dt)); IPN_LOG_DEBUG << "Reference angular velocity
                // [rad/s]: "
                //         << ref_omega.transpose();

                // if(ikey != opt_lens_traj + window_size - 1)
                // {
                //         input = result.at<gtsam::Vector4>(U(ikey + window_size - 1));
                //         IPN_LOG_DEBUG << "Optimized control input: "
                //                 << input.transpose();
                //         IPN_LOG_DEBUG << "Reference control input: "
                //                 << circle_generator.inputfm(t_ref).transpose();
                // }
                State m_state;
                m_state.p = i_pose.translation();
                if (ikey < window_size) {
                    state_trj.push_back(m_state);
                } else {
                    opt_trj.push_back(m_state);
                }

                IPN_LOG_DEBUG << "Trajectory: state_count=" << state_trj.size();
            }

            input = result.at<gtsam::Vector4>(U(window_size - 1));
            predicted_state.thrust_torque = input;
            predicted_state.timestamp = t0 + dt;
            last_state = predicted_state;
            quadrotor.setState(predicted_state);

            input = result.at<gtsam::Vector4>(U(window_size));
            quadrotor.stepODE(dt, input);
        } else {
            input = circle_generator.inputfm(t0 - dt);
            predicted_state.thrust_torque = input;
            predicted_state.timestamp = t0 + dt;
            last_state = predicted_state;
            quadrotor.setState(predicted_state);

            input = circle_generator.inputfm(t0);
            quadrotor.stepODE(dt, input);
        }

        predicted_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(predicted_state.rot, predicted_state.p);

        IPN_LOG_DEBUG << "Predicted position [m]: " << predicted_pose.translation();

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position = circle_generator.pos(t0 + 1 * dt);
        gtsam::Vector3 tar_theta = circle_generator.theta(t0 + 1 * dt);
        gtsam::Rot3 tar_rotation = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel = circle_generator.vel(t0 + 1 * dt);
        gtsam::Vector3 tar_omega = circle_generator.omega(t0 + 1 * dt);
        gtsam::Vector4 ref_input = circle_generator.inputfm(t0);

        gtsam::Vector3 err = predicted_state.p - tar_position;
        gtsam::Vector3 pred_theta = gtsam::Rot3::Logmap(predicted_state.rot);
        gtsam::Vector3 rot_err = tar_rotation.rpy() - predicted_state.rot.rpy();
        gtsam::Vector4 actuator_outputs = quadrotor.computeRotorVelocities();
        IPN_LOG_DEBUG << "Rendering trajectory: predicted_position_m="
                      << predicted_pose.translation();

        if (traj_idx < window_size) {
            if (!quadrotor.renderHistoryTrj()) {
                break;
            }
        } else {
            if (!quadrotor.renderHistoryOpt(opt_trj, err, landmarkk, vicon_measurement, rot_err,
                                            state_trj)) {
                break;
            }
        }

        IPN_LOG_DEBUG << "Trajectory rendered: predicted_position_m="
                      << predicted_pose.translation();

        /* real position, real attituede, real vel, rel augular speed, input their corr references
         */
        JEC_log << predicted_pose.translation().x() << " " << predicted_pose.translation().y()
                << " " << predicted_pose.translation().z() << " " << predicted_state.rot.rpy().x()
                << " " << predicted_state.rot.rpy().y() << " " << predicted_state.rot.rpy().z()
                << " " << predicted_state.v.x() << " " << predicted_state.v.y() << " "
                << predicted_state.v.z() << " " << predicted_state.body_rate.x() << " "
                << predicted_state.body_rate.y() << " " << predicted_state.body_rate.z() << " "
                << input[0] << " " << input[1] << " " << input[2] << " " << input[3] << " "
                << tar_position.x() << " " << tar_position.y() << " " << tar_position.z() << " "
                << tar_rotation.rpy().x() << " " << tar_rotation.rpy().y() << " "
                << tar_rotation.rpy().z() << " " << tar_vel.x() << " " << tar_vel.y() << " "
                << tar_vel.z() << " " << tar_omega.x() << " " << tar_omega.y() << " "
                << tar_omega.z() << " " << ref_input[0] << " " << ref_input[1] << " "
                << ref_input[2] << " " << ref_input[3];
    }

    while (quadrotor.renderHistoryTrj()) {
    }

    return 0;
}
