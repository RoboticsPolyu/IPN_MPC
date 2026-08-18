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
        argc > 1 ? argv[1] : "../config/factor_graph_hin.yaml";
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

    double high = FGO_config["clf_high"].as<double>();
    double low = FGO_config["clf_low"].as<double>();
    double thr = FGO_config["clf_thr"].as<double>();
    double alpha = FGO_config["clf_alpha"].as<double>();

    double input_jerk_t = FGO_config["input_jerk_t"].as<double>();
    double input_jerk_m = FGO_config["input_jerk_m"].as<double>();

    uint64_t sim_steps = FGO_config["sim_steps"].as<uint64_t>();

    std::string log_name = FGO_config["log_name"].as<std::string>();

    uint16_t window_size = FGO_config["window_size"].as<uint16_t>();

    double drag_force_x = FGO_config["drag_force_x"].as<double>();
    double drag_force_y = FGO_config["drag_force_y"].as<double>();
    double drag_force_z = FGO_config["drag_force_z"].as<double>();

    YAML::Node quad_config = YAML::LoadFile(quadrotor_config_path);

    double radius = quad_config["radius"].as<double>();
    double linear_vel = quad_config["linear_vel"].as<double>();
    double pos_meas_cov = quad_config["pos_meas_cov"].as<double>();
    double vel_meas_cov = quad_config["vel_meas_cov"].as<double>();
    double rot_meas_cov = quad_config["rot_meas_cov"].as<double>();
    double ome_meas_cov = quad_config["ome_meas_cov"].as<double>();
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

    double g_ = quad_config["g"].as<double>();
    double mass_ = quad_config["mass"].as<double>();
    double kf_ = quad_config["k_f"].as<double>(); //  xy-torque k-gain
    double km_ = quad_config["k_m"].as<double>(); // z-torque k-gain
    double motor_time_constant_ = quad_config["time_constant"].as<double>();

    double ixx = quad_config["ixx"].as<double>();
    double iyy = quad_config["iyy"].as<double>();
    double izz = quad_config["izz"].as<double>();
    gtsam::Vector3 Inertia = Eigen::Vector3d(ixx, iyy, izz);

    double rotor_px = 0.1f;
    double rotor_py = 0.1f;

    gtsam::Vector3 rotor_pos = gtsam::Vector3(rotor_px, rotor_py, 0);

    // double drag_force_x = quad_config["drag_force_x"].as<double>();
    // double drag_force_y = quad_config["drag_force_y"].as<double>();
    // double drag_force_z = quad_config["drag_force_z"].as<double>();

    gtsam::Vector3 drag_k = Eigen::Vector3d(drag_force_x, drag_force_y, drag_force_z);

    std::ofstream JEC_log;
    std::string file_name = "../data/JPC_";
    file_name.append(log_name);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    // double dt = 0.001f, radius = radius, linear_vel = linear_vel;
    // circle_generator circle_generator(radius, linear_vel, dt);

    double dt = 0.001, acc = 0.05;
    // circle_generator circle_generator(radius, linear_vel, dt);
    cir_conacc_generator circle_generator(radius, linear_vel, acc, dt);

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
    auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
    auto dynamics_noise = noiseModel::Diagonal::Sigmas(
        (Vector(12) << Vector3::Constant(dynamic_p_cov), Vector3::Constant(0.0001),
         Vector3::Constant(0.001), Vector3::Constant(0.001))
            .finished());

    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(rot_meas_cov), Vector3::Constant(pri_vicon_cov))
            .finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(
        Vector3(pri_vicon_vel_cov, pri_vicon_vel_cov, pri_vicon_vel_cov));
    auto omega_noise =
        noiseModel::Diagonal::Sigmas(Vector3(ome_meas_cov, ome_meas_cov, ome_meas_cov));

    auto ref_predict_vel_noise =
        noiseModel::Diagonal::Sigmas(Vector3(control_v_cov, control_v_cov, control_v_cov));
    auto ref_predict_omega_noise =
        noiseModel::Diagonal::Sigmas(Vector3(control_o_cov, control_o_cov, control_o_cov));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor(simulator_config_path);
    State predicted_state;
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;

    std::default_random_engine meas_rx_gen;
    std::default_random_engine meas_ry_gen;
    std::default_random_engine meas_rz_gen;

    std::default_random_engine meas_vx_gen;
    std::default_random_engine meas_vy_gen;
    std::default_random_engine meas_vz_gen;

    std::normal_distribution<double> position_noise(pos_meas_mean, pos_meas_cov);
    std::normal_distribution<double> rot_noise(0, rot_meas_cov);
    std::normal_distribution<double> velocity_noise(0, vel_meas_cov);

    std::ofstream state_log;
    file_name = "../data/simulated_state.txt";
    state_log.open(file_name);

    std::ofstream pwm_log;
    file_name = "../data/simulated_pwm.txt";
    pwm_log.open(file_name);

    Features landmarkk;
    Landmarks env(map_x, map_y, map_z, map_center_x, map_center_y, map_center_z, landmarks_size);
    Lidar<Landmarks> lidar(lidar_range, lidar_range_min);
    gtsam::Vector3 vicon_measurement;
    gtsam::Vector4 rotor_input_bak;

    double p_thrust_sigma = 0.10;
    gtsam::Vector3 thrust_sigma(0.01, 0.01, p_thrust_sigma);
    // sigma = (rotor_p_x* 2* P_thrust_single_rotor, rotor_p_x* 2* P_thrust_single_rotor, k_m * 2 *
    // P_thrust_single_rotor)
    gtsam::Vector3 torques_sigma(p_thrust_sigma * 2 * rotor_py, p_thrust_sigma * 2 * rotor_px,
                                 0.013f * 2 * p_thrust_sigma);

    auto dyn_noise = noiseModel::Diagonal::Sigmas((Vector(12) << thrust_sigma * 0.5f * dt * dt,
                                                   Vector3::Constant(0.01 * dt), thrust_sigma * dt,
                                                   torques_sigma * dt)
                                                      .finished());

    for (int traj_idx = 0; traj_idx < sim_steps; traj_idx++) {
        double t0 = traj_idx * dt;

        if (traj_idx == 0) {
            predicted_state.p = circle_generator.pos(t0);
            // predicted_state.rot          = gtsam::Rot3::identity(); //
            // gtsam::Rot3::Expmap(circle_generator.theta(t0)); predicted_state.v            =
            // gtsam::Vector3::Zero();  // circle_generator.vel(t0); predicted_state.body_rate    =
            // gtsam::Vector3::Zero(); // circle_generator.omega(t0);
            predicted_state.rot = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            predicted_state.v = circle_generator.vel(t0);
            predicted_state.body_rate = circle_generator.omega(t0);
            predicted_state.thrust_torque = circle_generator.inputfm(t0);
            quadrotor.setState(predicted_state);
        }

        if (traj_idx == 1000 && test_recovery) {
            predicted_state.p[0] = predicted_state.p[0] + move_x;
            predicted_state.p[1] = predicted_state.p[1] + move_y;
            predicted_state.p[2] = predicted_state.p[2] + move_z;
            quadrotor.setState(predicted_state);
        }

        NonlinearFactorGraph graph;
        Values initial_value;
        graph.empty();

        gtsam::Vector4 input_bak;

        for (int idx = 0; idx < opt_lens_traj; idx++) {
            DynamicFactor dynamics_factor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1),
                                          S(idx + 1), U(idx), dt, mass_, Inertia, rotor_pos, drag_k,
                                          kf_, km_, dyn_noise);
            graph.add(dynamics_factor);

            ControlLimitFactor control_limit_factor(U(idx), clf_sigma, low, high, thr, alpha);
            graph.add(control_limit_factor);

            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + (idx + 1) * dt)),
                                  circle_generator.pos(t0 + (idx + 1) * dt));
            gtsam::Vector3 vel_idx = circle_generator.vel(t0 + (idx + 1) * dt);
            gtsam::Vector3 omega_idx = circle_generator.omega(t0 + (idx + 1) * dt);

            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);
            initial_value.insert(S(idx + 1), omega_idx);

            gtsam::Vector4 init_input = circle_generator.input(t0 + idx * dt);
            if (idx != 0) {
                BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                graph.add(bet_FM_factor);
            }
            initial_value.insert(U(idx), init_input);
            // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));

            if (idx == opt_lens_traj - 1) {
                gtsam::Vector3 final_position_ref(control_p_final_cov_x, control_p_final_cov_y,
                                                  control_p_final_cov_z);
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << Vector3::Constant(control_r_cov), final_position_ref).finished());
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,
                // ref_predict_vel_noise)); graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1),
                // omega_idx, ref_predict_omega_noise)); auto correction_noise =
                // noiseModel::Isotropic::Sigma(3, control_p_final_cov_x); gtsam::GPSFactor
                // gps_factor(X(idx+1),
                //            Point3(pose_idx.translation()[0],   // N,
                //                   pose_idx.translation()[1],   // E,
                //                   pose_idx.translation()[2]),  // D,
                //            correction_noise);
                // graph.add(gps_factor);
            } else {
                gtsam::Vector3 _position_ref(control_p_cov_x, control_p_cov_y, control_p_cov_z);
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << Vector3::Constant(control_r_cov), _position_ref).finished());
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,
                // ref_predict_vel_noise)); graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1),
                // omega_idx, ref_predict_omega_noise)); auto correction_noise =
                // noiseModel::Isotropic::Sigma(3, control_p_cov_x); gtsam::GPSFactor
                // gps_factor(X(idx + 1),
                //            Point3(pose_idx.translation()[0],   // N,
                //                   pose_idx.translation()[1],   // E,
                //                   pose_idx.translation()[2]),  // D,
                //            correction_noise);
                // graph.add(gps_factor);
            }

            if (idx == 0) {
                gtsam::Vector3 pos_noise =
                    gtsam::Vector3(position_noise(meas_x_gen), position_noise(meas_y_gen),
                                   position_noise(meas_z_gen));
                gtsam::Vector3 vel_noise_add =
                    gtsam::Vector3(velocity_noise(meas_vx_gen), velocity_noise(meas_vy_gen),
                                   velocity_noise(meas_vz_gen));
                gtsam::Vector3 rot_noise_add = gtsam::Vector3(
                    rot_noise(meas_rx_gen), rot_noise(meas_ry_gen), rot_noise(meas_rz_gen));

                vicon_measurement = predicted_state.p + pos_noise;
                gtsam::Vector3 vel_add = predicted_state.v + vel_noise_add;
                gtsam::Vector3 rot_add = gtsam::Rot3::Logmap(predicted_state.rot) + rot_noise_add;

                graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                    X(idx), gtsam::Pose3(gtsam::Rot3::Expmap(rot_add), vicon_measurement),
                    vicon_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), vel_add, vel_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), predicted_state.body_rate,
                                                             omega_noise));

                initial_value.insert(X(idx), gtsam::Pose3(predicted_state.rot, vicon_measurement));
                initial_value.insert(V(idx), vel_add);
                initial_value.insert(S(idx), predicted_state.body_rate);
            }
        }

        IPN_LOG_DEBUG << "Optimizer: constructing";
        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        IPN_LOG_DEBUG << "Optimizer: starting solve";
        Values result = optimizer.optimize();

        std::vector<State> opt_trj;

        gtsam::Pose3 i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < opt_lens_traj; ikey++) {
            IPN_LOG_DEBUG << "Trajectory optimization: state_index=" << ikey;
            i_pose = result.at<Pose3>(X(ikey));
            vel = result.at<Vector3>(V(ikey));
            omega = result.at<Vector3>(S(ikey));
            gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t0 + ikey * dt)),
                                  circle_generator.pos(t0 + ikey * dt));
            gtsam::Vector3 ref_vel = circle_generator.vel(t0 + ikey * dt);
            gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt);

            // IPN_LOG_DEBUG << "Optimized position [m]: "
            //         << i_pose.translation();
            // IPN_LOG_DEBUG << "Reference position [m]: "
            //         << ref_pose.translation();
            // IPN_LOG_DEBUG << "Optimized rotation logmap [rad]: "
            //         << Rot3::Logmap(i_pose.rotation()).transpose();
            // IPN_LOG_DEBUG << "Reference rotation logmap [rad]: "
            //         << Rot3::Logmap(ref_pose.rotation()).transpose();
            // IPN_LOG_DEBUG << "Optimized velocity [m/s]: "
            //         << vel.transpose();
            // //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey *
            // dt)); IPN_LOG_DEBUG << "Reference velocity [m/s]: "
            //         << ref_vel.transpose();
            // IPN_LOG_DEBUG << "Optimized angular velocity [rad/s]: "
            //         << omega.transpose();
            //  //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey
            //  * dt));
            // IPN_LOG_DEBUG << "Reference angular velocity [rad/s]: "
            //         << ref_omega.transpose();

            if (ikey != opt_lens_traj - 1) {
                input = result.at<gtsam::Vector4>(U(ikey));
                // IPN_LOG_DEBUG << "Optimized control input: "
                //         << input.transpose();
                // IPN_LOG_DEBUG << "Reference control input: "
                //         << circle_generator.inputfm(t0 + ikey * dt).transpose();
            }
            State m_state;
            m_state.p = i_pose.translation();
            opt_trj.push_back(m_state);
        }

        input = result.at<gtsam::Vector4>(U(0));

        for (int j = 0; j < 4; j++) {
            if (input[j] < 0)
                return 0;
        }
        gtsam::Vector4 tt = quadrotor.inverseRotorVelocities(input);

        // quadrotor.setInput(input);

        // IPN_LOG_DEBUG << "planned input: " << input;
        // gtsam::Vector4 actuator_outputs = quadrotor.CumputeRotorsVel();

        // float  Tc = 0.100f;
        // float  T  = 0.01f;
        // float  a1, a2;
        // double fc = 1/ (2* M_PI* Tc);
        // a1        = 1.0 / (1+ 2* M_PI* fc* T);
        // a2        = 2* M_PI* fc* T/ (1+ 2* M_PI* fc* T);

        // if(traj_idx != 0)
        // {
        //     actuator_outputs = a1* rotor_input_bak + a2* actuator_outputs;
        // }

        // predicted_state.thrust_torque = input_bak;
        // predicted_state.timestamp = t0 + dt;
        // quadrotor.setState(predicted_state);

        // rotor_input_bak = actuator_outputs;
        // input = quadrotor.InvCumputeRotorsVel(actuator_outputs);
        // input_bak = input;

        // IPN_LOG_DEBUG << "actuator_outputs: " << actuator_outputs;
        // predicted_state.thrust_torque = input;
        // predicted_state.timestamp = t0 + dt;
        // quadrotor.setState(predicted_state);

        // input = result.at<gtsam::Vector4>(U(1));
        // quadrotor.stepODE(dt, result.at<gtsam::Vector4>(U(0)));

        quadrotor.stepODE(dt, tt); // for driver delay test

        IPN_LOG_DEBUG << "Control input: trajectory_index=" << traj_idx
                      << ", value=" << input.transpose();

        predicted_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(predicted_state.rot, predicted_state.p);

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position = circle_generator.pos(t0 + 1 * dt);
        gtsam::Vector3 tar_theta = circle_generator.theta(t0 + 1 * dt);
        gtsam::Rot3 tar_rotation = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel = circle_generator.vel(t0 + 1 * dt);
        gtsam::Vector3 tar_omega = circle_generator.omega(t0 + 1 * dt);
        gtsam::Vector4 ref_input = circle_generator.input(t0);

        gtsam::Vector3 err = predicted_state.p - tar_position;
        gtsam::Vector3 pred_theta = gtsam::Rot3::Logmap(predicted_state.rot);
        gtsam::Vector3 rot_err = tar_rotation.rpy() - predicted_state.rot.rpy();
        // actuator_outputs = quadrotor.CumputeRotorsVel();

        if (!quadrotor.renderHistoryOpt(opt_trj, err, landmarkk, vicon_measurement, rot_err)) {
            break;
        }

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
