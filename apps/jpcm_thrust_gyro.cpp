#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <ipn_mpc/simulation/landmarks.h>
#include <ipn_mpc/simulation/lidar.h>
#include <ipn_mpc/trajectory/trajectory_generator.h>
#include <time.h>
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
    clock_t start, end;

    const std::string factor_graph_config_path =
        argc > 1 ? argv[1] : "../config/factor_graph_thrust_gyro.yaml";
    const std::string quadrotor_config_path =
        argc > 2 ? argv[2] : "../config/quadrotor_thrust_gyro.yaml";
    IPN_LOG_INFO << "factor_graph_config=" << factor_graph_config_path
                 << " quadrotor_config=" << quadrotor_config_path;

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
    double control_r1_cov = FGO_config["control_r1_cov"].as<double>();
    double control_r2_cov = FGO_config["control_r2_cov"].as<double>();
    double control_r3_cov = FGO_config["control_r3_cov"].as<double>();
    uint16_t opt_lens_traj = FGO_config["opt_lens_traj"].as<uint16_t>();

    double prior_u_f_cov = FGO_config["prior_u_f_cov"].as<double>();
    double prior_u_m1_cov = FGO_config["prior_u_m1_cov"].as<double>();
    double prior_u_m2_cov = FGO_config["prior_u_m2_cov"].as<double>();
    double prior_u_m3_cov = FGO_config["prior_u_m3_cov"].as<double>();

    double input_jerk_t = FGO_config["input_jerk_t"].as<double>();
    double input_jerk_m = FGO_config["input_jerk_m"].as<double>();
    double input_jerk_m3 = FGO_config["input_jerk_m3"].as<double>();

    uint64_t sim_steps = FGO_config["sim_steps"].as<uint64_t>();

    std::string log_name = FGO_config["log_name"].as<std::string>();

    uint16_t window_size = FGO_config["window_size"].as<uint16_t>();

    double high = FGO_config["clf_high"].as<double>();
    double low = FGO_config["clf_low"].as<double>();
    double thr = FGO_config["clf_thr"].as<double>();
    double ghigh = FGO_config["g_clf_high"].as<double>();
    double glow = FGO_config["g_clf_low"].as<double>();
    double gthr = FGO_config["g_clf_thr"].as<double>();
    double alpha = FGO_config["clf_alpha"].as<double>();

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
    double mass = 1.0f;
    gtsam::Vector3 drag_k(-0.0, -0., -0.);
    // gtsam::Vector3 drag_k(-0.20, -0.23, -0.31);

    std::ofstream JEC_log;
    std::string file_name = "../data/log/JPC_TGyro_";
    file_name.append(log_name);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    double dt = 0.001f;
    circle_generator fig_gen(radius, linear_vel, dt);

    // figure_eight_generator fig_gen(radius, linear_vel, dt);  // scale=1m, speed=1rad/s, dt=0.01s

    traj_state state;
    state.t = 0.0;
    state.pos = fig_gen.pos(0.0);
    state.vel = fig_gen.vel(0.0);
    state.rotation = gtsam::Rot3::Expmap(fig_gen.theta(0.0)).toQuaternion();
    state.angular_speed = fig_gen.omega(0.0);
    state.acc = fig_gen.thrust(0.0);
    state.motor = fig_gen.input(0.0);

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 100;
    parameters.relativeErrorTol = 1e-2;
    parameters.maxIterations = 10;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;

    auto input_jerk = noiseModel::Diagonal::Sigmas(
        Vector4(input_jerk_t, input_jerk_m, input_jerk_m, input_jerk_m3));
    auto input_noise = noiseModel::Diagonal::Sigmas(
        Vector4(prior_u_f_cov, prior_u_m1_cov, prior_u_m2_cov, prior_u_m3_cov));

    auto dynamics_noise =
        noiseModel::Diagonal::Sigmas((Vector(9) << Vector3::Constant(dynamic_p_cov),
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

    auto point_obs_noise = noiseModel::Diagonal::Sigmas(Vector1(0.001));

    auto ref_predict_vel_noise =
        noiseModel::Diagonal::Sigmas(Vector3(control_v_cov, control_v_cov, control_v_cov));
    auto ref_predict_omega_noise =
        noiseModel::Diagonal::Sigmas(Vector3(control_o_cov, control_o_cov, control_o_cov));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor(quadrotor_config_path);
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
    file_name = "../data/log/simulated_state.txt";
    state_log.open(file_name);

    std::ofstream pwm_log;
    file_name = "../data/log/simulated_state.txt";
    pwm_log.open(file_name);

    Features landmarkk;
    Landmarks env(map_x, map_y, map_z, map_center_x, map_center_y, map_center_z, landmarks_size);
    Lidar<Landmarks> lidar(lidar_range, lidar_range_min);
    gtsam::Vector3 vicon_measurement;
    gtsam::Vector4 rotor_input_bak;
    gtsam::Vector3 obs1(0, 0, 0);
    std::vector<gtsam::Vector3> obstacles;

    float obs1_radius = 0.20f, safe_d = 0.10f;

    for (int traj_idx = 0; traj_idx < sim_steps; traj_idx++) {
        std::vector<State> opt_trj, ref_trj;
        obs1 = quadrotor.getObs1();
        // obstacles = quadrotor.getObstacles();
        // IPN_LOG_DEBUG << " Obstacles size is " << obstacles.size();
        double t0 = traj_idx * dt;

        if (traj_idx == 0) {
            // predicted_state.p            = circle_generator.pos(t0) - gtsam::Vector3(0,0,1);
            // // gtsam::Vector3 rzyx(0, 0, 10.0/180.0*3.14159);
            // // gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(rzyx);
            // gtsam::Rot3 rot = gtsam::Rot3::identity();
            // predicted_state.rot          = rot; //
            // gtsam::Rot3::Expmap(circle_generator.theta(t0)); predicted_state.v            =
            // gtsam::Vector3::Zero();  // circle_generator.vel(t0); predicted_state.body_rate    =
            // gtsam::Vector3::Zero(); // circle_generator.omega(t0);
            // // predicted_state.rot          = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            // // predicted_state.v            = circle_generator.vel(t0);
            // // predicted_state.body_rate    = circle_generator.omega(t0);
            // predicted_state.thrust_torque = circle_generator.inputfm(t0);

            // state.t = 0.0;
            // state.pos = fig_gen.pos(0.0);
            // state.vel = fig_gen.vel(0.0);
            // state.rotation = gtsam::Rot3::Expmap(fig_gen.theta(0.0)).toQuaternion();
            // state.angular_speed = fig_gen.omega(0.0);
            // state.acc = fig_gen.thrust(0.0);
            // state.motor = fig_gen.input(0.0);

            predicted_state.p = fig_gen.pos(0.0);
            gtsam::Rot3 rot = gtsam::Rot3::Expmap(fig_gen.theta(0.0)).toQuaternion();
            predicted_state.rot = rot;
            predicted_state.v = fig_gen.vel(0.0);
            predicted_state.body_rate = fig_gen.omega(0.0);
            // predicted_state.thrust_torque = circle_generator.inputfm(t0);

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
        auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
        ControlLimitTGyroFactor cntrolLimitTGyroFactor(U(0), clf_sigma, low, high, glow, ghigh, thr,
                                                       gthr, alpha);
        graph.add(cntrolLimitTGyroFactor);

        for (int idx = 0; idx < opt_lens_traj; idx++) {
            DynamicsFactorTGyro dynamics_factor(X(idx), V(idx), U(idx), X(idx + 1), V(idx + 1), dt,
                                                mass, drag_k, dynamics_noise);
            graph.add(dynamics_factor);

            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(fig_gen.theta(t0 + (idx + 1) * dt)),
                                  fig_gen.pos(t0 + (idx + 1) * dt));

            gtsam::Vector3 vel_idx = fig_gen.vel(t0 + (idx + 1) * dt);
            float hvel = 0.50f;
            // gtsam::Pose3 pose_idx;
            // gtsam::Vector3 vel_idx;

            // if((t0 + float(idx+1)/100.0) < 4)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, 0.0 + (t0 +
            //     float(idx+1)/100.0)* hvel)); vel_idx = gtsam::Vector3(0, 0, hvel);
            // }
            // else if((t0 + float(idx+1)/100.0) > 4 && (t0 + float(idx+1)/100.0) < 8)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.30, 0.40, 0.0 + (t0 +
            //     float(idx+1)/100.0)* hvel)); vel_idx = gtsam::Vector3(0, 0, hvel);
            // }
            // else if((t0 + float(idx+1)/100.0) >= 8 && (t0 + float(idx+1)/100.0) < 12)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.30, 0.40, 0.0 + 8*
            //     hvel)); vel_idx = gtsam::Vector3(0, 0, 0);
            // }
            // else if((t0 + float(idx+1)/100.0) >= 12 && (t0 + float(idx+1)/100.0) <= 16)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.50, 0.70, 0.0 + 8* hvel -
            //     ((t0 + float(idx+1)/100.0) - 12)* hvel)); vel_idx = gtsam::Vector3(0, 0, -hvel);
            // }
            // else
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.50, 0.70, 0.0 + 8* hvel -
            //     4* hvel)); vel_idx = gtsam::Vector3(0, 0, 0);
            // }

            gtsam::Vector3 omega_idx = fig_gen.omega(t0 + (idx + 1) * dt);

            State ref_state;
            ref_state.p = pose_idx.translation();
            ref_state.rot = pose_idx.rotation();
            ref_state.v = vel_idx;
            ref_trj.push_back(ref_state);

            gtsam::Vector4 init_input(10, 0, 0, 0);
            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);
            initial_value.insert(U(idx), init_input);

            // gtsam::Vector4 init_input = circle_generator.inputfm(t0 + idx * dt);
            if (idx != 0) {
                BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                graph.add(bet_FM_factor);
            }
            // initial_value.insert(U(idx), init_input); //
            // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            gtsam::Vector3 control_r_cov(control_r1_cov, control_r2_cov, control_r3_cov);
            if (idx == opt_lens_traj - 1) {
                gtsam::Vector3 final_position_ref(control_p_final_cov_x, control_p_final_cov_y,
                                                  control_p_final_cov_z);
                // auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) <<
                // Vector3::Constant(control_r_cov), final_position_ref).finished());
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << control_r_cov, final_position_ref).finished());

                // float d2 = std::sqrt((pose_idx.translation() - obs1).transpose()*
                // (pose_idx.translation() - obs1));

                // if(d2 < obs1_radius)
                // {
                //     float scale = obs1_radius / d2;
                //     pose_idx = gtsam::Pose3(pose_idx.rotation(), (pose_idx.translation() - obs1)*
                //     scale + obs1); graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1),
                //     pose_idx, ref_predict_pose_noise));
                //     graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,
                //     ref_predict_vel_noise));
                // }
                // else
                // {
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                graph.add(
                    gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                // }

                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx,
                // ref_predict_omega_noise)); auto correction_noise =
                // noiseModel::Isotropic::Sigma(3, control_p_final_cov_x); gtsam::GPSFactor
                // gps_factor(X(idx+1),
                //            Point3(pose_idx.translation()[0],   // N,
                //                   pose_idx.translation()[1],   // E,
                //                   pose_idx.translation()[2]),  // D,
                //            correction_noise);
                // graph.add(gps_factor);
                // for(uint16_t obsi = 0; obsi < obstacles.size(); obsi++)
                // {
                //     obs1 = obstacles[obsi];
                //     graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d,
                //     point_obs_noise));
                // }
            } else {
                gtsam::Vector3 _position_ref(control_p_cov_x, control_p_cov_y, control_p_cov_z);
                // auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) <<
                // Vector3::Constant(control_r_cov), _position_ref).finished());
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << control_r_cov, _position_ref).finished());

                // float d2 = std::sqrt((pose_idx.translation() - obs1).transpose()*
                // (pose_idx.translation() - obs1)); if(d2 < obs1_radius)
                // {
                //     float scale = obs1_radius / d2;
                //     pose_idx = gtsam::Pose3(pose_idx.rotation(), (pose_idx.translation() - obs1)*
                //     scale + obs1);
                // }
                // if((pose_idx.translation() - obs1).transpose()* (pose_idx.translation() - obs1)
                // >= 0.50 * 0.50)
                // {
                //     graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx,
                //     ref_predict_pose_noise)); graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx
                //     + 1), vel_idx, ref_predict_vel_noise));
                // }
                //   else
                //    {
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                graph.add(
                    gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));

                //    }
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx,
                // ref_predict_omega_noise)); auto correction_noise =
                // noiseModel::Isotropic::Sigma(3, control_p_cov_x); gtsam::GPSFactor
                // gps_factor(X(idx + 1),
                //            Point3(pose_idx.translation()[0],   // N,
                //                   pose_idx.translation()[1],   // E,
                //                   pose_idx.translation()[2]),  // D,
                //            correction_noise);
                // graph.add(gps_factor);

                // for(uint16_t obsi=0; obsi < obstacles.size(); obsi++)
                // {
                //     obs1 = obstacles[obsi];
                //     graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d,
                //     point_obs_noise));
                // }
            }

            // for(uint16_t obsi = 0; obsi < obstacles.size(); obsi++)
            // {
            //     obs1 = obstacles[obsi];
            //     graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d, point_obs_noise));
            // }

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
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), predicted_state.body_rate,
                // omega_noise));

                IPN_LOG_DEBUG << "Measurement position noise [m]: " << pos_noise.transpose();

                initial_value.insert(X(idx), gtsam::Pose3(predicted_state.rot, vicon_measurement));
                initial_value.insert(V(idx), vel_add);
                // initial_value.insert(S(idx), predicted_state.body_rate);
            }
        }

        IPN_LOG_DEBUG << "Optimizer: constructing";
        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);
        BatchFixedLagSmoother smoother(7.0, LevenbergMarquardtParams());

        IPN_LOG_DEBUG << "Optimizer: starting solve";
        start = clock();
        Values result = optimizer.optimize();
        end = clock();
        float opt_cost = (double)(end - start) / CLOCKS_PER_SEC;
        IPN_LOG_DEBUG << "Optimizer: solve_time_ms=" << opt_cost;

        gtsam::Pose3 i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < opt_lens_traj; ikey++) {
            IPN_LOG_DEBUG << "Trajectory optimization: state_index=" << ikey;
            i_pose = result.at<Pose3>(X(ikey));
            vel = result.at<Vector3>(V(ikey));
            // omega = result.at<Vector3>(S(ikey));
            // gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt);
            // gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t0 + ikey * dt)),
            // circle_generator.pos(t0 + ikey * dt)); gtsam::Vector3 ref_vel =
            // circle_generator.vel(t0 + ikey * dt);

            IPN_LOG_DEBUG << "Optimized position [m]: " << i_pose.translation().transpose();
            // IPN_LOG_DEBUG << "Reference position [m]: "
            //         << ref_pose.translation();

            IPN_LOG_DEBUG << "Optimized rotation logmap [rad]: "
                          << Rot3::Logmap(i_pose.rotation()).transpose();
            // IPN_LOG_DEBUG << "Reference rotation logmap [rad]: "
            //         << Rot3::Logmap(ref_pose.rotation()).transpose();

            IPN_LOG_DEBUG << "Optimized velocity [m/s]: " << vel.transpose();
            //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey *
            // dt));
            // IPN_LOG_DEBUG << "Reference velocity [m/s]: "
            //         << ref_vel.transpose();

            // IPN_LOG_DEBUG << "Optimized angular velocity [rad/s]: "
            //         << omega.transpose();
            //  //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey
            //  * dt));
            // IPN_LOG_DEBUG << "Reference angular velocity [rad/s]: "
            //         << ref_omega.transpose();

            if (ikey != opt_lens_traj - 1) {
                input = result.at<gtsam::Vector4>(U(ikey));
                IPN_LOG_DEBUG << "Optimized control input: " << input.transpose();
                // IPN_LOG_DEBUG << "Reference control input: "
                //         << circle_generator.inputfm(t0 + ikey * dt).transpose();
            }
            State m_state;
            m_state.p = i_pose.translation();
            opt_trj.push_back(m_state);
        }

        input = result.at<gtsam::Vector4>(U(0));

        IPN_LOG_DEBUG << "Control input: value=" << input.transpose();

        /* Simulator */
        State est_state = quadrotor.getState();
        gtsam::Vector3 drag_force = est_state.rot.matrix() * Eigen::Matrix3d(drag_k.asDiagonal()) *
                                    est_state.rot.matrix().transpose() * est_state.v;

        float g_ = 9.81f;
        std::normal_distribution<double> thrust_noise(0, 0.2);
        std::normal_distribution<double> gyro_noise(0, 0.02);
        std::default_random_engine generator_;
        double at_noise = thrust_noise(generator_);
        double wx_noise = gyro_noise(generator_);
        double wy_noise = gyro_noise(generator_);
        double wz_noise = gyro_noise(generator_);

        gtsam::Vector3 v_dot =
            -gtsam::Vector3(0, 0, g_) +
            est_state.rot.rotate(gtsam::Vector3(0, 0, (input[0] + at_noise) / mass)) +
            drag_force / mass;

        gtsam::Vector3 p_dot = est_state.v;

        Eigen::Matrix3d r_dot = est_state.rot.matrix() * gtsam::skewSymmetric(est_state.body_rate);

        est_state.p = est_state.p + p_dot * dt;
        est_state.v = est_state.v + v_dot * dt;
        gtsam::Vector3 body_rate = gtsam::Vector3(input[1], input[2], input[3]) +
                                   gtsam::Vector3(wx_noise, wy_noise, wz_noise);
        est_state.rot = est_state.rot * gtsam::Rot3::Expmap(body_rate * dt);

        quadrotor.setState(est_state);

        predicted_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(predicted_state.rot, predicted_state.p);

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position = fig_gen.pos(t0 + 1 * dt);
        gtsam::Vector3 tar_theta = fig_gen.theta(t0 + 1 * dt);
        gtsam::Rot3 tar_rotation = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel = fig_gen.vel(t0 + 1 * dt);
        gtsam::Vector3 tar_omega = fig_gen.omega(t0 + 1 * dt);
        gtsam::Vector4 ref_input = fig_gen.inputfm(t0);

        gtsam::Vector3 pos_err = predicted_state.p - tar_position;
        gtsam::Vector3 pred_theta = gtsam::Rot3::Logmap(predicted_state.rot);
        gtsam::Vector3 rot_err = tar_rotation.rpy() - predicted_state.rot.rpy();

        IPN_LOG_DEBUG << "Target attitude RPY [rad]: " << tar_rotation.rpy().transpose();
        IPN_LOG_DEBUG << "Predicted attitude RPY [rad]: " << predicted_state.rot.rpy().transpose();
        // actuator_outputs = quadrotor.CumputeRotorsVel();

        // quadrotor.renderHistoryOpt(opt_trj, err, landmarkk, vicon_measurement, rot_err);
        if (!quadrotor.renderHistoryOpt(opt_trj, pos_err, boost::none, vicon_measurement, rot_err,
                                        ref_trj, opt_cost)) {
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
                << ref_input[2] << " " << ref_input[3] << " " << opt_cost << " ";
    }

    while (quadrotor.renderHistoryTrj()) {
    }

    return 0;
}
