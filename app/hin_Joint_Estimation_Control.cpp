#include "color.h"
#include "env_sensors_sim/Landmarks.h"
#include "env_sensors_sim/Lidar.h"
#include "quadrotor_simulator/Dynamics_control_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"
#include "trajectory_generator/Trajectory_generator.h"

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

int main(void)
{
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier green(Color::FG_GREEN);
    

    // Configuration file 
    YAML::Node FGO_config        = YAML::LoadFile("../config/factor_graph_hin.yaml");  
    double PRI_VICON_COV         = FGO_config["PRI_VICON_COV"].as<double>();
    double PRI_VICON_VEL_COV     = FGO_config["PRI_VICON_VEL_COV"].as<double>();   
    double CONTROL_P_COV_X       = FGO_config["CONTROL_P_COV_X"].as<double>();
    double CONTROL_P_COV_Y       = FGO_config["CONTROL_P_COV_Y"].as<double>();
    double CONTROL_P_COV_Z       = FGO_config["CONTROL_P_COV_Z"].as<double>();
    double CONTROL_P_FINAL_COV_X = FGO_config["CONTROL_P_FINAL_COV_X"].as<double>();
    double CONTROL_P_FINAL_COV_Y = FGO_config["CONTROL_P_FINAL_COV_Y"].as<double>();
    double CONTROL_P_FINAL_COV_Z = FGO_config["CONTROL_P_FINAL_COV_Z"].as<double>();
    
    double CONTROL_O_COV         = FGO_config["CONTROL_O_COV"].as<double>();

    double CONTROL_V_COV         = FGO_config["CONTROL_V_COV"].as<double>();
    double DYNAMIC_P_COV         = FGO_config["DYNAMIC_P_COV"].as<double>(); 
    double CONTROL_R_COV         = FGO_config["CONTROL_R_COV"].as<double>();
    uint16_t OPT_LENS_TRAJ       = FGO_config["OPT_LENS_TRAJ"].as<uint16_t>();

    double PRIOR_U_F_COV         = FGO_config["PRIOR_U_F_COV"].as<double>(); 
    double PRIOR_U_M1_COV        = FGO_config["PRIOR_U_M1_COV"].as<double>(); 
    double PRIOR_U_M2_COV        = FGO_config["PRIOR_U_M2_COV"].as<double>(); 
    double PRIOR_U_M3_COV        = FGO_config["PRIOR_U_M3_COV"].as<double>(); 
    
    double high        = FGO_config["CLF_HIGH"].as<double>(); 
    double low         = FGO_config["CLF_LOW"].as<double>(); 
    double thr         = FGO_config["CLF_THR"].as<double>(); 
    double alpha       = FGO_config["CLF_ALPHA"].as<double>(); 

    double INPUT_JERK_T          = FGO_config["INPUT_JERK_T"].as<double>(); 
    double INPUT_JERK_M          = FGO_config["INPUT_JERK_M"].as<double>(); 

    uint64_t SIM_STEPS           = FGO_config["SIM_STEPS"].as<uint64_t>();

    std::string LOG_NAME         = FGO_config["LOG_NAME"].as<std::string>();
    
    uint16_t WINDOW_SIZE         = FGO_config["WINDOW_SIZE"].as<uint16_t>();

    double DRAG_FORCE_X = FGO_config["DRAG_FORCE_X"].as<double>();
    double DRAG_FORCE_Y = FGO_config["DRAG_FORCE_Y"].as<double>();
    double DRAG_FORCE_Z = FGO_config["DRAG_FORCE_Z"].as<double>();

    YAML::Node quad_config  = YAML::LoadFile("../config/quadrotor.yaml"); 

    double RADIUS                = quad_config["RADIUS"].as<double>();
    double LINEAR_VEL            = quad_config["LINEAR_VEL"].as<double>();
    double POS_MEAS_COV          = quad_config["POS_MEAS_COV"].as<double>();
    double VEL_MEAS_COV          = quad_config["VEL_MEAS_COV"].as<double>();
    double ROT_MEAS_COV          = quad_config["ROT_MEAS_COV"].as<double>();
    double OME_MEAS_COV          = quad_config["OME_MEAS_COV"].as<double>();
    double POS_MEAS_MEAN         = quad_config["POS_MEAS_MEAN"].as<double>();
    bool   TEST_RECOVERY         = quad_config["TEST_RECOVERY"].as<bool>();

    double MAP_X                 = quad_config["MAP_X"].as<double>();
    double MAP_Y                 = quad_config["MAP_Y"].as<double>();
    double MAP_Z                 = quad_config["MAP_Z"].as<double>();

    double MAP_CENTER_X          = quad_config["MAP_CENTER_X"].as<double>();
    double MAP_CENTER_Y          = quad_config["MAP_CENTER_Y"].as<double>();
    double MAP_CENTER_Z          = quad_config["MAP_CENTER_Z"].as<double>();
    double LIDAR_RANGE           = quad_config["LIDAR_RANGE"].as<double>();
    double LIDAR_RANGE_MIN       = quad_config["LIDAR_RANGE_MIN"].as<double>();
    double LANDMARKS_SIZE        = quad_config["LANDMARKS_SIZE"].as<uint32_t>();
    
    double MOVE_X                = quad_config["MOVE_X"].as<double>();
    double MOVE_Y                = quad_config["MOVE_Y"].as<double>();
    double MOVE_Z                = quad_config["MOVE_Z"].as<double>();

    double g_                   = quad_config["g"].as<double>();
    double mass_                = quad_config["mass"].as<double>();
    double kf_                  = quad_config["k_f"].as<double>(); //  xy-moment k-gain
    double km_                  = quad_config["k_m"].as<double>(); // z-moment k-gain
    double motor_time_constant_ = quad_config["time_constant"].as<double>();

    double Ixx                  = quad_config["Ixx"].as<double>();
    double Iyy                  = quad_config["Iyy"].as<double>();
    double Izz                  = quad_config["Izz"].as<double>();
    gtsam::Vector3 Inertia = Eigen::Vector3d(Ixx, Iyy, Izz);
    
    double rotor_px = 0.1f;
    double rotor_py = 0.1f;

    gtsam::Vector3 rotor_pos = gtsam::Vector3(rotor_px, rotor_py, 0);
    
    // double DRAG_FORCE_X = quad_config["DRAG_FORCE_X"].as<double>();
    // double DRAG_FORCE_Y = quad_config["DRAG_FORCE_Y"].as<double>();
    // double DRAG_FORCE_Z = quad_config["DRAG_FORCE_Z"].as<double>();
    
    gtsam::Vector3 drag_k = Eigen::Vector3d(DRAG_FORCE_X, DRAG_FORCE_Y, DRAG_FORCE_Z);

    std::ofstream JEC_log;
    std::string file_name = "../data/JPC_";
    file_name.append(LOG_NAME);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    // double dt = 0.001f, radius = RADIUS, linear_vel = LINEAR_VEL;
    // circle_generator circle_generator(radius, linear_vel, dt);

    double dt = 0.001, radius = RADIUS, linear_vel = 5.0, acc = 0.05;
    // circle_generator circle_generator(radius, linear_vel, dt);
    cir_conacc_generator circle_generator(radius, linear_vel, acc, dt);

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(INPUT_JERK_T, INPUT_JERK_M, INPUT_JERK_M, INPUT_JERK_M));
    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(PRIOR_U_F_COV, PRIOR_U_M1_COV, PRIOR_U_M2_COV, PRIOR_U_M3_COV));
    auto clf_sigma   = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(DYNAMIC_P_COV), Vector3::Constant(0.0001), 
        Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());
    
    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(ROT_MEAS_COV), Vector3::Constant(PRI_VICON_COV)).finished());
    auto vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(PRI_VICON_VEL_COV, PRI_VICON_VEL_COV, PRI_VICON_VEL_COV));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(OME_MEAS_COV, OME_MEAS_COV, OME_MEAS_COV));

    auto ref_predict_vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(CONTROL_V_COV, CONTROL_V_COV, CONTROL_V_COV));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(CONTROL_O_COV, CONTROL_O_COV, CONTROL_O_COV));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor;
    Quadrotor::State predicted_state;
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;
    
    std::default_random_engine meas_rx_gen;
    std::default_random_engine meas_ry_gen;
    std::default_random_engine meas_rz_gen;

    std::default_random_engine meas_vx_gen;
    std::default_random_engine meas_vy_gen;
    std::default_random_engine meas_vz_gen;

    std::normal_distribution<double> position_noise(POS_MEAS_MEAN, POS_MEAS_COV);
    std::normal_distribution<double> rot_noise(0, ROT_MEAS_COV);
    std::normal_distribution<double> velocity_noise(0, VEL_MEAS_COV);
    
    std::ofstream state_log;
    file_name = "../data/simulated_state.txt";
    state_log.open(file_name);

    std::ofstream pwm_log;
    file_name = "../data/simulated_state.txt";
    pwm_log.open(file_name);

    Features landmarkk; 
    Landmarks env(MAP_X, MAP_Y, MAP_Z, MAP_CENTER_X, MAP_CENTER_Y, MAP_CENTER_Z, LANDMARKS_SIZE);
    Lidar<Landmarks> lidar(LIDAR_RANGE, LIDAR_RANGE_MIN);
    gtsam::Vector3 vicon_measurement;
    gtsam::Vector4 rotor_input_bak;
    
    double p_thrust_sigma = 0.10;
    gtsam::Vector3 thrust_sigma(0.01, 0.01, p_thrust_sigma);
    // sigma = (rotor_p_x* 2* P_thrust_single_rotor, rotor_p_x* 2* P_thrust_single_rotor, k_m * 2 * P_thrust_single_rotor) 
    gtsam::Vector3 moments_sigma(p_thrust_sigma * 2 * rotor_py, p_thrust_sigma * 2 * rotor_px, 0.013f * 2 * p_thrust_sigma);

    auto dyn_noise = noiseModel::Diagonal::Sigmas((Vector(12) << thrust_sigma * 0.5f * dt * dt, Vector3::Constant(0.01 * dt), 
                                                                   thrust_sigma * dt, moments_sigma * dt).finished());

    for(int traj_idx = 0; traj_idx < SIM_STEPS; traj_idx++)
    {
        double t0 = traj_idx* dt;

        if(traj_idx == 0)
        {
            predicted_state.p            = circle_generator.pos(t0);
            // predicted_state.rot          = gtsam::Rot3::identity(); // gtsam::Rot3::Expmap(circle_generator.theta(t0));
            // predicted_state.v            = gtsam::Vector3::Zero();  // circle_generator.vel(t0);
            // predicted_state.body_rate    = gtsam::Vector3::Zero(); // circle_generator.omega(t0);
            predicted_state.rot          = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            predicted_state.v            = circle_generator.vel(t0);
            predicted_state.body_rate    = circle_generator.omega(t0);
            predicted_state.thrust_torque = circle_generator.inputfm(t0);
            quadrotor.setState(predicted_state);
        }
        
        if(traj_idx == 1000 && TEST_RECOVERY)
        {
            predicted_state.p[0] = predicted_state.p[0] + MOVE_X;
            predicted_state.p[1] = predicted_state.p[1] + MOVE_Y;
            predicted_state.p[2] = predicted_state.p[2] + MOVE_Z;
            quadrotor.setState(predicted_state);
        }

        NonlinearFactorGraph graph;
        Values initial_value;
        graph.empty();

        gtsam::Vector4 input_bak;

        for (int idx = 0; idx < OPT_LENS_TRAJ; idx++)
        {
            DynamicFactor dynamics_factor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), U(idx), 
                dt, mass_, Inertia, rotor_pos, drag_k, kf_, km_, dyn_noise);
            graph.add(dynamics_factor);
            
            ControlLimitFactor control_limit_factor(U(idx), clf_sigma, low, high, thr, alpha);
            graph.add(control_limit_factor);

            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + (idx + 1) * dt)), circle_generator.pos(t0 + (idx + 1) * dt));
            gtsam::Vector3 vel_idx = circle_generator.vel(t0 + (idx + 1) * dt);
            gtsam::Vector3 omega_idx = circle_generator.omega(t0 + (idx + 1) * dt);
            
            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);
            initial_value.insert(S(idx + 1), omega_idx);
            
            gtsam::Vector4 init_input = circle_generator.input(t0 + idx * dt);
            if(idx != 0)
            {
                BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                graph.add(bet_FM_factor);
            }
            initial_value.insert(U(idx), init_input);
            // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            
            if(idx == OPT_LENS_TRAJ - 1)
            {
                gtsam::Vector3 final_position_ref(CONTROL_P_FINAL_COV_X, CONTROL_P_FINAL_COV_Y, CONTROL_P_FINAL_COV_Z);
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), final_position_ref).finished());   
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
                // auto correction_noise = noiseModel::Isotropic::Sigma(3, CONTROL_P_FINAL_COV_X);
                // gtsam::GPSFactor gps_factor(X(idx+1),
                //            Point3(pose_idx.translation()[0],   // N,
                //                   pose_idx.translation()[1],   // E,
                //                   pose_idx.translation()[2]),  // D,
                //            correction_noise);
                // graph.add(gps_factor);
            }
            else
            {
                gtsam::Vector3 _position_ref(CONTROL_P_COV_X, CONTROL_P_COV_Y, CONTROL_P_COV_Z);
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), _position_ref).finished());
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
                // auto correction_noise = noiseModel::Isotropic::Sigma(3, CONTROL_P_COV_X);
                // gtsam::GPSFactor gps_factor(X(idx + 1),
                //            Point3(pose_idx.translation()[0],   // N,
                //                   pose_idx.translation()[1],   // E,
                //                   pose_idx.translation()[2]),  // D,
                //            correction_noise);
                // graph.add(gps_factor);
            }

            if (idx == 0)
            {                
                gtsam::Vector3 pos_noise     = gtsam::Vector3(position_noise(meas_x_gen), position_noise(meas_y_gen), position_noise(meas_z_gen));
                gtsam::Vector3 vel_noise_add = gtsam::Vector3(velocity_noise(meas_vx_gen), velocity_noise(meas_vy_gen), velocity_noise(meas_vz_gen));
                gtsam::Vector3 rot_noise_add = gtsam::Vector3(rot_noise(meas_rx_gen), rot_noise(meas_ry_gen), rot_noise(meas_rz_gen));
                
                vicon_measurement      = predicted_state.p + pos_noise;
                gtsam::Vector3 vel_add = predicted_state.v + vel_noise_add;
                gtsam::Vector3 rot_add = gtsam::Rot3::Logmap(predicted_state.rot) + rot_noise_add;

                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), gtsam::Pose3(gtsam::Rot3::Expmap(rot_add), vicon_measurement), vicon_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), vel_add, vel_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), predicted_state.body_rate, omega_noise));

                initial_value.insert(X(idx), gtsam::Pose3(predicted_state.rot, vicon_measurement));
                initial_value.insert(V(idx), vel_add);
                initial_value.insert(S(idx), predicted_state.body_rate);
            }

        }

        std::cout << "###################### init optimizer ######################" << std::endl;
        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        std::cout << "###################### begin optimize ######################" << std::endl;
        Values result = optimizer.optimize();

        std::vector<Quadrotor::State> opt_trj;

        gtsam::Pose3   i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < OPT_LENS_TRAJ; ikey++)
        {
                std::cout << red << "--------------------------------- TRAJECTORY CONTROL OPTIMIZATION: "  << ikey << " ----------------------------------" << def << std::endl;
                i_pose = result.at<Pose3>(X(ikey));
                vel = result.at<Vector3>(V(ikey));
                omega = result.at<Vector3>(S(ikey));
                gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t0 + ikey * dt)), circle_generator.pos(t0 + ikey * dt));
                gtsam::Vector3 ref_vel = circle_generator.vel(t0 + ikey * dt); 
                gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt);
                
                
                // std::cout << green << "OPT Translation: "
                //         << i_pose.translation() << std::endl;
                // std::cout << "REF Translation: "
                //         << ref_pose.translation() << std::endl;
                // std::cout << "OPT    Rotation: "
                //         << Rot3::Logmap(i_pose.rotation()).transpose() << std::endl;
                // std::cout << "REF    Rotation: "
                //         << Rot3::Logmap(ref_pose.rotation()).transpose() << std::endl;
                // std::cout << "OPT         VEL: "
                //         << vel.transpose() << std::endl;
                // //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                // std::cout << "REF         VEL: "
                //         << ref_vel.transpose() << std::endl;
                // std::cout << "OPT       OMEGA: "
                //         << omega.transpose() << std::endl;
                //  //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                // std::cout << "REF       OMEGA: "
                //         << ref_omega.transpose() << std::endl;

                if(ikey != OPT_LENS_TRAJ - 1)
                {
                        input = result.at<gtsam::Vector4>(U(ikey));
                        // std::cout << "OPT      INPUT: "
                        //         << input.transpose() << std::endl;
                        // std::cout << "REF      INPUT: "
                        //         << circle_generator.inputfm(t0 + ikey * dt).transpose() << std::endl;
                }
                Quadrotor::State m_state;
                m_state.p = i_pose.translation();
                opt_trj.push_back(m_state);

        }

        input = result.at<gtsam::Vector4>(U(0));

        for(int j = 0; j < 4; j++)
        {
            if(input[j] < 0)
                return 0;
        }
        gtsam::Vector4 tt = quadrotor.InvCumputeRotorsVel(input);

        // quadrotor.setInput(input);

        // std::cout << "planned input: " << input << std::endl;
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

        // std::cout << "actuator_outputs: " << actuator_outputs << std::endl;
        // predicted_state.thrust_torque = input;
        // predicted_state.timestamp = t0 + dt;
        // quadrotor.setState(predicted_state);
        
        // input = result.at<gtsam::Vector4>(U(1));
        // quadrotor.stepODE(dt, result.at<gtsam::Vector4>(U(0)));

        quadrotor.stepODE(dt, tt); // for driver delay test

        std::cout << "RPM Input [ " << traj_idx << " ]" << input.transpose() << std::endl;

        predicted_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(predicted_state.rot, predicted_state.p);

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position     = circle_generator.pos(t0 + 1 * dt);
        gtsam::Vector3 tar_theta        = circle_generator.theta(t0 + 1 * dt);
        gtsam::Rot3    tar_rotation     = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel          = circle_generator.vel(t0 + 1 * dt);
        gtsam::Vector3 tar_omega        = circle_generator.omega(t0 + 1 * dt);
        gtsam::Vector4 ref_input        = circle_generator.input(t0);

        gtsam::Vector3 err              = predicted_state.p - tar_position;
        gtsam::Vector3 pred_theta       = gtsam::Rot3::Logmap(predicted_state.rot);
        gtsam::Vector3 rot_err          = tar_rotation.rpy() - predicted_state.rot.rpy();
        // actuator_outputs = quadrotor.CumputeRotorsVel();
        
        quadrotor.renderHistoryOpt(opt_trj, err, landmarkk, vicon_measurement, rot_err);

        /* real position, real attituede, real vel, rel augular speed, input their corr references */
        JEC_log << predicted_pose.translation().x() << " " << predicted_pose.translation().y() << " " << predicted_pose.translation().z() << " " 
            << predicted_state.rot.rpy().x() << " " << predicted_state.rot.rpy().y() << " " << predicted_state.rot.rpy().z() << " " 
            << predicted_state.v.x() << " " << predicted_state.v.y() << " " << predicted_state.v.z() << " "
            << predicted_state.body_rate.x() << " " << predicted_state.body_rate.y() << " " << predicted_state.body_rate.z() << " " 
            << input[0] << " " << input[1] << " " << input[2] << " " << input[3] << " "
            << tar_position.x() << " " << tar_position.y() << " " << tar_position.z() << " "
            << tar_rotation.rpy().x() << " " << tar_rotation.rpy().y() << " " << tar_rotation.rpy().z() << " "
            << tar_vel.x() << " " << tar_vel.y() << " " << tar_vel.z() << " "
            << tar_omega.x() << " " << tar_omega.y() << " " << tar_omega.z() << " "
            << ref_input[0] << " " << ref_input[1] << " " << ref_input[2] << " " << ref_input[3] << std::endl;
    }

    while (true)
    {
        quadrotor.renderHistoryTrj();
    }

    return 0;
}
