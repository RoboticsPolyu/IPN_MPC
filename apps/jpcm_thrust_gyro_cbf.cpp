#include <algorithm>
#include <gtsam_unstable/linear/QPSolver.h>
#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/cbf_factor.h>
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

namespace {

constexpr double kMinimumDistance = 1.0e-6;
constexpr double kBoundaryEpsilon = 1.0e-3;

gtsam::Values solveHardConstrainedTrajectory(const gtsam::NonlinearFactorGraph& graph,
                                             const gtsam::Values& initial_values,
                                             const std::vector<Obstacle>& obstacles,
                                             std::size_t horizon_length, double time_step,
                                             double safety_margin, double vehicle_radius,
                                             std::size_t sqp_iterations) {
    gtsam::Values values = initial_values;

    for (std::size_t iteration = 0; iteration < sqp_iterations; ++iteration) {
        const auto linear_cost = graph.linearize(values);
        gtsam::InequalityFactorGraph inequalities;
        struct HalfSpace {
            gtsam::Key key;
            gtsam::RowVector coefficient;
            double upper_bound;
        };
        std::vector<HalfSpace> half_spaces;
        std::size_t constraint_index = 0;

        for (std::size_t step = 1; step <= horizon_length; ++step) {
            const gtsam::Key pose_key = X(step);
            const gtsam::Pose3& pose = values.at<gtsam::Pose3>(pose_key);
            gtsam::Matrix36 translation_jacobian;
            const gtsam::Point3 position = pose.translation(translation_jacobian);

            for (const Obstacle& obstacle : obstacles) {
                const gtsam::Vector3 obstacle_position =
                    obstacle.obs_pos + obstacle.obs_vel * (step * time_step);
                gtsam::Vector3 separation = position - obstacle_position;

                if (obstacle.obs_type == ObsType::cylinder) {
                    separation.z() = 0.0;
                } else if (obstacle.obs_type != ObsType::sphere) {
                    continue;
                }

                const double distance = separation.norm();
                gtsam::Vector3 outward_normal = gtsam::Vector3::UnitX();
                if (distance > kMinimumDistance) {
                    outward_normal = separation / distance;
                }
                const double required_clearance =
                    obstacle.obs_size + safety_margin + vehicle_radius;

                // Supporting half-space of the obstacle boundary:
                // n' * (p + J*delta - obstacle) >= required_clearance.
                const gtsam::RowVector pose_coefficient =
                    -outward_normal.transpose() * translation_jacobian;
                const double upper_bound = distance - required_clearance;
                const gtsam::Key dual_key = gtsam::Symbol('q', constraint_index++);
                inequalities.add(pose_key, pose_coefficient, upper_bound, dual_key);
                half_spaces.push_back({pose_key, pose_coefficient, upper_bound});
            }
        }

        if (inequalities.empty()) {
            return values;
        }

        const gtsam::QP quadratic_program(*linear_cost, gtsam::EqualityFactorGraph{}, inequalities);
        const gtsam::QPSolver solver(quadratic_program);
        gtsam::VectorValues feasible_delta = values.zeroVectors();

        // The unstable GTSAM QP initializer can stall on some feasible problems. Construct a
        // feasible point deterministically with cyclic projections onto all linearized half-spaces.
        for (std::size_t projection_iteration = 0; projection_iteration < 100;
             ++projection_iteration) {
            double maximum_violation = 0.0;
            for (const HalfSpace& half_space : half_spaces) {
                gtsam::Vector& pose_delta = feasible_delta[half_space.key];
                const double violation =
                    (half_space.coefficient * pose_delta)(0) - half_space.upper_bound;
                maximum_violation = std::max(maximum_violation, violation);
                if (violation > 0.0) {
                    pose_delta -= (violation / half_space.coefficient.squaredNorm()) *
                                  half_space.coefficient.transpose();
                }
            }
            if (maximum_violation <= 1.0e-9) {
                break;
            }
        }

        const gtsam::VectorValues delta = solver.optimize(feasible_delta).first;
        values = values.retract(delta);
    }

    return values;
}

void enforceSafetyBoundary(State& state, const std::vector<Obstacle>& obstacles, double time_step,
                           double safety_margin, double vehicle_radius) {
    for (const Obstacle& obstacle : obstacles) {
        const gtsam::Vector3 obstacle_position = obstacle.obs_pos + obstacle.obs_vel * time_step;
        gtsam::Vector3 separation = state.p - obstacle_position;

        if (obstacle.obs_type == ObsType::cylinder) {
            separation.z() = 0.0;
        } else if (obstacle.obs_type != ObsType::sphere) {
            continue;
        }

        const double required_clearance = obstacle.obs_size + safety_margin + vehicle_radius;
        const double distance = separation.norm();
        if (distance >= required_clearance) {
            continue;
        }

        gtsam::Vector3 outward_normal;
        if (distance > kMinimumDistance) {
            outward_normal = separation / distance;
        } else {
            outward_normal = gtsam::Vector3::UnitX();
        }

        const double penetration = required_clearance + kBoundaryEpsilon - distance;
        state.p += penetration * outward_normal;

        const gtsam::Vector3 relative_velocity = state.v - obstacle.obs_vel;
        const double inward_speed = relative_velocity.dot(outward_normal);
        if (inward_speed < 0.0) {
            state.v -= inward_speed * outward_normal;
        }

        IPN_LOG_WARNING << "Safety boundary activated: obstacle_type="
                        << static_cast<int>(obstacle.obs_type) << ", distance_m=" << distance
                        << ", required_clearance_m=" << required_clearance;
    }
}

} // namespace

int main(int argc, char** argv) {
    clock_t start, end;

    const std::string factor_graph_config_path =
        argc > 1 ? argv[1] : "../config/factor_graph_thrust_gyro_cbf.yaml";
    const std::string quadrotor_config_path =
        argc > 2 ? argv[2] : "../config/quadrotor_thrust_gyro_cbf.yaml";
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
    double dynamic_t_cov = FGO_config["dynamic_t_cov"].as<double>();
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
    double point_obs_sigma = FGO_config["point_obs_sigma"].as<double>();
    double high = FGO_config["clf_high"].as<double>();
    double low = FGO_config["clf_low"].as<double>();
    double thr = FGO_config["clf_thr"].as<double>();
    double ghigh = FGO_config["g_clf_high"].as<double>();
    double glow = FGO_config["g_clf_low"].as<double>();
    double gthr = FGO_config["g_clf_thr"].as<double>();
    double alpha = FGO_config["clf_alpha"].as<double>();
    uint8_t maxIterations = FGO_config["max_iters"].as<double>();
    double cbf_alpha = FGO_config["cbf_alpha"].as<double>();
    double cbf_beta = FGO_config["cbf_beta"].as<double>();
    const bool use_hard_constraints = FGO_config["use_hard_constraints"].as<bool>(true);
    const std::size_t hard_constraint_iterations =
        FGO_config["hard_constraint_iterations"].as<std::size_t>(3);

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
    double safe_d = quad_config["safe_d"].as<double>();
    double quad_size = quad_config["uav_size"].as<double>();

    double map_center_x = quad_config["map_center_x"].as<double>();
    double map_center_y = quad_config["map_center_y"].as<double>();
    double map_center_z = quad_config["map_center_z"].as<double>();
    double lidar_range = quad_config["lidar_range"].as<double>();
    double lidar_range_min = quad_config["lidar_range_min"].as<double>();
    double landmarks_size = quad_config["landmarks_size"].as<uint32_t>();

    double move_x = quad_config["move_x"].as<double>();
    double move_y = quad_config["move_y"].as<double>();
    double move_z = quad_config["move_z"].as<double>();
    double drag_x = quad_config["drag_force_x"].as<double>();
    double drag_y = quad_config["drag_force_y"].as<double>();
    double drag_z = quad_config["drag_force_z"].as<double>();


    double mass = 1.0f;

    gtsam::Vector3 drag_k(drag_x, drag_y, drag_z);

    std::ofstream JEC_log;
    std::string file_name = "../data/log/JPC_TGyro_";
    file_name.append(log_name);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    double dt = 0.001f;
    // Trajectory::circle_generator fig_gen(radius, linear_vel, dt);
    Trajectory::back_and_forth_generator fig_gen(5.0, 2.0, 0.001);
    // Trajectory::figure_eight_generator fig_gen(radius, linear_vel, dt);  // scale=1m,
    // speed=1rad/s, dt=0.01s

    traj_state state;
    state.t = 0.0;
    state.pos = fig_gen.pos(0.0);
    state.vel = fig_gen.vel(0.0);
    state.rotation = gtsam::Rot3::Expmap(fig_gen.theta(0.0)).toQuaternion();

    state.angular_speed = gtsam::Vector3::Zero();
    state.acc = fig_gen.thrust(0.0);
    state.motor = fig_gen.input(0.0);

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 100;
    parameters.relativeErrorTol = 1e-2;
    parameters.maxIterations = maxIterations;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;

    auto input_jerk = noiseModel::Diagonal::Sigmas(
        Vector4(input_jerk_t, input_jerk_m, input_jerk_m, input_jerk_m3));
    auto input_noise = noiseModel::Diagonal::Sigmas(
        Vector4(prior_u_f_cov, prior_u_m1_cov, prior_u_m2_cov, prior_u_m3_cov));
    float dyn_dt = 0.01f;
    auto dynamics_noise = noiseModel::Diagonal::Sigmas(
        (Vector(9) << Vector3::Constant(dynamic_t_cov * 0.5 * dyn_dt * dyn_dt),
         Vector3::Constant(dynamic_t_cov * dyn_dt), Vector3::Constant(0.01))
            .finished());

    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(rot_meas_cov), Vector3::Constant(pri_vicon_cov))
            .finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(
        Vector3(pri_vicon_vel_cov, pri_vicon_vel_cov, pri_vicon_vel_cov));
    auto omega_noise =
        noiseModel::Diagonal::Sigmas(Vector3(ome_meas_cov, ome_meas_cov, ome_meas_cov));

    auto point_obs_noise = noiseModel::Diagonal::Sigmas(Vector1(point_obs_sigma));

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
    Obstacle obs1;
    std::vector<Obstacle> obstacles;

    gtsam::Matrix3 _E12;
    _E12 << 1, 0, 0, 0, 1, 0, 0, 0, 0;
    float quad_radius = quad_size / 2.0;
    for (int traj_idx = 0; traj_idx < sim_steps; traj_idx++) {
        obstacles = quadrotor.getObstacles();
        double t0 = traj_idx * dt;
        std::vector<State> opt_trj, ref_trj;
        State ref_state;

        // if(traj_idx == 0)
        // {
        //                 predicted_state.p         = fig_gen.pos(0.0);
        //     gtsam::Rot3 rot                       =
        //     gtsam::Rot3::Expmap(fig_gen.theta(0.0)).toQuaternion();
        //                 predicted_state.rot       = rot;
        //                 predicted_state.v         = fig_gen.vel(0.0);
        //                 predicted_state.body_rate = fig_gen.omega(0.0);

        //     quadrotor.setState(predicted_state);
        // }

        if (traj_idx == 0) {
            State init_state;
            init_state.p = fig_gen.pos(0.0);
            init_state.rot = gtsam::Rot3::Expmap(fig_gen.theta(0.0));
            init_state.v = fig_gen.vel(0.0);
            init_state.body_rate = fig_gen.omega(0.0);

            quadrotor.setState(init_state);
            predicted_state = init_state;

            // Submit the first Pangolin frame before the optimizer starts so the window is never
            // left blank during optimization initialization.
            if (!quadrotor.renderHistoryTrj()) {
                return 0;
            }
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
            gtsam::Vector3 omega_idx = fig_gen.omega(t0 + (idx + 1) * dt);

            ref_state.p = pose_idx.translation();
            ref_state.rot = pose_idx.rotation();
            ref_state.v = vel_idx;
            ref_trj.push_back(ref_state);

            gtsam::Vector4 init_input(10, 0, 0, 0);
            initial_value.insert(U(idx), init_input);

            for (uint16_t obsi = 0; obsi < obstacles.size(); obsi++) {
                obs1 = obstacles[obsi];
                const gtsam::Vector3 obs_pos_linear_vel =
                    obs1.obs_pos + obs1.obs_vel * (idx + 1) * dt;
                if (obs1.obs_type == ObsType::sphere) {
                    const double distance = (pose_idx.translation() - obs_pos_linear_vel).norm();

                    if (distance < obs1.obs_size + safe_d + quad_radius) {
                        const double safe_distance = std::max(distance, kMinimumDistance);
                        const double scale = (obs1.obs_size + safe_d + quad_radius) / safe_distance;
                        pose_idx =
                            gtsam::Pose3(pose_idx.rotation(),
                                         (pose_idx.translation() - obs_pos_linear_vel) * scale +
                                             obs_pos_linear_vel);
                    }
                } else if (obs1.obs_type == ObsType::cylinder) {
                    const double distance =
                        (_E12 * (pose_idx.translation() - obs_pos_linear_vel)).norm();

                    if (distance < obs1.obs_size + safe_d + quad_radius) {
                        const double safe_distance = std::max(distance, kMinimumDistance);
                        const double scale = (obs1.obs_size + safe_d + quad_radius) / safe_distance;

                        gtsam::Point3 current_pos = pose_idx.translation();
                        gtsam::Point3 obs_pos = obs_pos_linear_vel;

                        gtsam::Point3 new_xy_pos(
                            (current_pos.x() - obs_pos.x()) * scale + obs_pos.x(),
                            (current_pos.y() - obs_pos.y()) * scale + obs_pos.y(), current_pos.z());

                        pose_idx = gtsam::Pose3(pose_idx.rotation(), new_xy_pos);
                    }
                }
            }

            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);

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
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << control_r_cov, final_position_ref).finished());
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                graph.add(
                    gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
            } else {
                gtsam::Vector3 _position_ref(control_p_cov_x, control_p_cov_y, control_p_cov_z);
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << control_r_cov, _position_ref).finished());
                graph.add(
                    gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                graph.add(
                    gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
            }

            for (uint16_t obsi = 0; obsi < obstacles.size(); obsi++) {
                obs1 = obstacles[obsi];

                const gtsam::Vector3 obs_pos_linear_vel =
                    obs1.obs_pos + obs1.obs_vel * (idx + 1) * dt;
                if (obs1.obs_type == ObsType::sphere) {
                    // graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d,
                    // point_obs_noise));
                    if (idx == opt_lens_traj - 1) {
                        graph.add(CBFPdFactor(X(idx + 1), V(idx + 1), obs_pos_linear_vel,
                                              obs1.obs_size + safe_d + quad_radius, cbf_alpha,
                                              point_obs_noise));
                    } else {
                        graph.add(VeCBFPdFactor1(X(idx + 1), V(idx + 1), U(idx), obs_pos_linear_vel,
                                                 obs1.obs_vel, obs1.obs_size + safe_d + quad_radius,
                                                 cbf_alpha, cbf_beta, point_obs_noise));
                    }
                } else if (obs1.obs_type == ObsType::cylinder) {
                    // IPN_LOG_DEBUG << "Cylinder Obs pos: " << obs1.obs_pos.transpose() << " - vel:
                    // " << obs1.obs_vel.transpose();
                    if (idx == opt_lens_traj - 1) {
                        graph.add(CBFPdFactorCylinder(X(idx + 1), V(idx + 1), obs_pos_linear_vel,
                                                      obs1.obs_size + safe_d + quad_radius,
                                                      cbf_alpha, point_obs_noise));
                    } else {
                        graph.add(VeCBFPdFactorCylinder1(X(idx + 1), V(idx + 1), U(idx),
                                                         obs_pos_linear_vel, obs1.obs_vel,
                                                         obs1.obs_size + safe_d + quad_radius,
                                                         cbf_alpha, cbf_beta, point_obs_noise));
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

                initial_value.insert(X(idx), gtsam::Pose3(predicted_state.rot, vicon_measurement));
                initial_value.insert(V(idx), vel_add);
                // initial_value.insert(S(idx), predicted_state.body_rate);
            }
        }

        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);
        BatchFixedLagSmoother smoother(7.0, LevenbergMarquardtParams());

        IPN_LOG_DEBUG << "Optimizer: starting solve";
        start = clock();
        Values result = optimizer.optimize();
        if (use_hard_constraints) {
            try {
                result =
                    solveHardConstrainedTrajectory(graph, result, obstacles, opt_lens_traj, dt,
                                                   safe_d, quad_radius, hard_constraint_iterations);
            } catch (const std::exception& exception) {
                IPN_LOG_ERROR << "Hard-constrained SQP failed; using soft-CBF solution: reason="
                              << exception.what();
            }
        }
        end = clock();
        float opt_cost = (float)(end - start) / CLOCKS_PER_SEC;
        IPN_LOG_DEBUG << "Optimizer: solve_time_ms=" << opt_cost;

        gtsam::Pose3 ipose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < opt_lens_traj; ikey++) {
            IPN_LOG_DEBUG << "Trajectory optimization: state_index=" << ikey;
            ipose = result.at<Pose3>(X(ikey));
            vel = result.at<Vector3>(V(ikey));

            // omega = result.at<Vector3>(S(ikey));
            // gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt);
            gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(fig_gen.theta(t0 + ikey * dt)),
                                  fig_gen.pos(t0 + ikey * dt));
            gtsam::Vector3 ref_vel = fig_gen.vel(t0 + ikey * dt);

            IPN_LOG_DEBUG << "Optimized position [m]: " << ipose.translation();
            IPN_LOG_DEBUG << "Reference position [m]: " << ref_pose.translation();

            IPN_LOG_DEBUG << "Optimized rotation logmap [rad]: "
                          << Rot3::Logmap(ipose.rotation()).transpose();
            IPN_LOG_DEBUG << "Reference rotation logmap [rad]: "
                          << Rot3::Logmap(ref_pose.rotation()).transpose();

            IPN_LOG_DEBUG << "Optimized velocity [m/s]: " << vel.transpose();
            (gtsam::Rot3::Expmap(fig_gen.theta(ikey * dt)), fig_gen.pos(ikey * dt));
            IPN_LOG_DEBUG << "Reference velocity [m/s]: " << ref_vel.transpose();

            // IPN_LOG_DEBUG << "Optimized angular velocity [rad/s]: "
            //         << omega.transpose();
            //  //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey
            //  * dt));
            // IPN_LOG_DEBUG << "Reference angular velocity [rad/s]: "
            //         << ref_omega.transpose();

            if (ikey != opt_lens_traj - 1) {
                input = result.at<gtsam::Vector4>(U(ikey));
                IPN_LOG_DEBUG << "Optimized control input: " << input.transpose();
                IPN_LOG_DEBUG << "Reference control input: "
                              << fig_gen.inputfm(t0 + ikey * dt).transpose();
            }
            State m_state;
            m_state.p = ipose.translation();
            opt_trj.push_back(m_state);
        }

        input = result.at<gtsam::Vector4>(U(0));

        // IPN_LOG_DEBUG << "Control input: value=" << input.transpose();

        /* Simulator */
        State est_state = quadrotor.getState();
        gtsam::Vector3 drag_force = est_state.rot.matrix() * Eigen::Matrix3d(drag_k.asDiagonal()) *
                                    est_state.rot.matrix().transpose() * est_state.v;
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
            est_state.rot.rotate(gtsam::Vector3(0, 0, (input[0] + at_noise) / mass)) +
            drag_force / mass;
        gtsam::Vector3 p_dot = est_state.v;
        Eigen::Matrix3d r_dot = est_state.rot.matrix() * gtsam::skewSymmetric(est_state.body_rate);

        est_state.p = est_state.p + p_dot * dt;
        est_state.v = est_state.v + v_dot * dt;
        gtsam::Vector3 body_rate = gtsam::Vector3(input[1], input[2], input[3]) +
                                   gtsam::Vector3(wx_noise, wy_noise, wz_noise);
        est_state.rot = est_state.rot * gtsam::Rot3::Expmap(body_rate * dt);

        enforceSafetyBoundary(est_state, obstacles, dt, safe_d, quad_radius);

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

        if (!quadrotor.renderHistoryOpt(opt_trj, pos_err, boost::none, vicon_measurement, rot_err,
                                        ref_trj, opt_cost)) {
            break;
        }

        if (obstacles.size() == 1) {
            JEC_log << predicted_pose.translation().x() << " " << predicted_pose.translation().y()
                    << " " << predicted_pose.translation().z() << " "
                    << predicted_state.rot.rpy().x() << " " << predicted_state.rot.rpy().y() << " "
                    << predicted_state.rot.rpy().z() << " " << predicted_state.v.x() << " "
                    << predicted_state.v.y() << " " << predicted_state.v.z() << " "
                    << predicted_state.body_rate.x() << " " << predicted_state.body_rate.y() << " "
                    << predicted_state.body_rate.z() << " " << input[0] << " " << input[1] << " "
                    << input[2] << " " << input[3] << " " << tar_position.x() << " "
                    << tar_position.y() << " " << tar_position.z() << " " << tar_rotation.rpy().x()
                    << " " << tar_rotation.rpy().y() << " " << tar_rotation.rpy().z() << " "
                    << tar_vel.x() << " " << tar_vel.y() << " " << tar_vel.z() << " "
                    << tar_omega.x() << " " << tar_omega.y() << " " << tar_omega.z() << " "
                    << ref_input[0] << " " << ref_input[1] << " " << ref_input[2] << " "
                    << ref_input[3] << " " << opt_cost << " " << obstacles[0].obs_pos.x() << " "
                    << obstacles[0].obs_pos.y() << " " << obstacles[0].obs_pos.z() << " "
                    << obstacles[0].obs_vel.x() << " " << obstacles[0].obs_vel.y() << " "
                    << obstacles[0].obs_vel.z() << " ";
        } else {
            /* real position, real attituede, real vel, rel augular speed, input their corr
             * references */
            JEC_log << predicted_pose.translation().x() << " " << predicted_pose.translation().y()
                    << " " << predicted_pose.translation().z() << " "
                    << predicted_state.rot.rpy().x() << " " << predicted_state.rot.rpy().y() << " "
                    << predicted_state.rot.rpy().z() << " " << predicted_state.v.x() << " "
                    << predicted_state.v.y() << " " << predicted_state.v.z() << " "
                    << predicted_state.body_rate.x() << " " << predicted_state.body_rate.y() << " "
                    << predicted_state.body_rate.z() << " " << input[0] << " " << input[1] << " "
                    << input[2] << " " << input[3] << " " << tar_position.x() << " "
                    << tar_position.y() << " " << tar_position.z() << " " << tar_rotation.rpy().x()
                    << " " << tar_rotation.rpy().y() << " " << tar_rotation.rpy().z() << " "
                    << tar_vel.x() << " " << tar_vel.y() << " " << tar_vel.z() << " "
                    << tar_omega.x() << " " << tar_omega.y() << " " << tar_omega.z() << " "
                    << ref_input[0] << " " << ref_input[1] << " " << ref_input[2] << " "
                    << ref_input[3] << " " << opt_cost << " ";
        }
    }

    while (quadrotor.renderHistoryTrj()) {
    }

    return 0;
}
