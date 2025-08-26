#include "color.h"
#include "env_sensors_sim/Landmarks.h"
#include "env_sensors_sim/Lidar.h"
#include "control/Dynamics_control_factor.h"
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
    YAML::Node FGO_config        = YAML::LoadFile("../config/factor_graph.yaml");  
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
    
    double INPUT_JERK_T          = FGO_config["INPUT_JERK_T"].as<double>(); 
    double INPUT_JERK_M          = FGO_config["INPUT_JERK_M"].as<double>(); 

    uint64_t SIM_STEPS           = FGO_config["SIM_STEPS"].as<uint64_t>();

    std::string LOG_NAME         = FGO_config["LOG_NAME"].as<std::string>();
    
    uint16_t WINDOW_SIZE         = FGO_config["WINDOW_SIZE"].as<uint16_t>();

    
    YAML::Node quad_config  = YAML::LoadFile("../config/quadrotor.yaml"); 

    double RADIUS                = quad_config["RADIUS"].as<double>();
    double LINEAR_VEL            = quad_config["LINEAR_VEL"].as<double>();
    double POS_MEAS_COV          = quad_config["POS_MEAS_COV"].as<double>();
    double VEL_MEAS_COV          = quad_config["VEL_MEAS_COV"].as<double>();
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


    std::ofstream JEC_log;
    std::string file_name = "../data/JEC_";
    file_name.append(LOG_NAME);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    double dt = 0.001f, radius = RADIUS, linear_vel = LINEAR_VEL;
    circle_generator circle_generator(radius, linear_vel, dt);

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations = 500;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(INPUT_JERK_T, INPUT_JERK_M, INPUT_JERK_M, INPUT_JERK_M));
    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(PRIOR_U_F_COV, PRIOR_U_M1_COV, PRIOR_U_M2_COV, PRIOR_U_M3_COV));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(DYNAMIC_P_COV), Vector3::Constant(0.0005), 
        Vector3::Constant(0.0005), Vector3::Constant(0.0005)).finished());
    
    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(PRI_VICON_COV)).finished());
    auto vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(PRI_VICON_VEL_COV, PRI_VICON_VEL_COV, PRI_VICON_VEL_COV));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(CONTROL_V_COV, CONTROL_V_COV, CONTROL_V_COV));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(CONTROL_O_COV, CONTROL_O_COV, CONTROL_O_COV));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor;
    State last_state, predicted_state;
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;
    
    std::default_random_engine meas_vx_gen;
    std::default_random_engine meas_vy_gen;
    std::default_random_engine meas_vz_gen;

    std::default_random_engine meas_lidar_gen;

    std::normal_distribution<double> position_noise(POS_MEAS_MEAN, POS_MEAS_COV);
    std::normal_distribution<double> velocity_noise(0, VEL_MEAS_COV);
    std::normal_distribution<double> lidar_noise(0, 0.003);

    Features landmarkk; 
    Landmarks env(MAP_X, MAP_Y, MAP_Z, MAP_CENTER_X, MAP_CENTER_Y, MAP_CENTER_Z, LANDMARKS_SIZE);
    Lidar<Landmarks> lidar(LIDAR_RANGE, LIDAR_RANGE_MIN);
    gtsam::Vector3 vicon_measurement;

    std::vector<State> measurements;
    std::vector<gtsam::Pose3>     lidar_measures;
    std::vector<State> opt_trj, state_trj;

    for(int traj_idx = 0; traj_idx < SIM_STEPS; traj_idx++)
    {
        double t0 = traj_idx* dt;

        if(traj_idx == 0)
        {
            predicted_state.p            =  circle_generator.pos(t0);
            predicted_state.rot          = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            predicted_state.v            = circle_generator.vel(t0);
            predicted_state.body_rate        = circle_generator.omega(t0);
            predicted_state.thrust_torque = circle_generator.inputfm(t0);
            quadrotor.setState(predicted_state);
        }
        
        if(traj_idx == 50 && TEST_RECOVERY)
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

        gtsam::Vector3 pos_noise     = gtsam::Vector3(position_noise(meas_x_gen), position_noise(meas_y_gen), position_noise(meas_z_gen));
        gtsam::Vector3 vel_noise_add = gtsam::Vector3(velocity_noise(meas_vx_gen), velocity_noise(meas_vy_gen), velocity_noise(meas_vz_gen));
        gtsam::Vector6 lidar_noise_add;
        lidar_noise_add << lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen), lidar_noise(meas_lidar_gen);
        
        vicon_measurement      = predicted_state.p + pos_noise;
        gtsam::Vector3 vel_add = predicted_state.v + vel_noise_add;
        
        // vicon_measurement = predicted_state.p;
        // gtsam::Vector3 vel_add = predicted_state.v;

        State m_state;
        m_state.p     = vicon_measurement;
        m_state.rot   = predicted_state.rot;
        m_state.v     = vel_add;
        m_state.body_rate = predicted_state.body_rate;

        measurements.push_back(m_state);


        if(traj_idx > 0)
        {
            gtsam::Pose3 lidar_pR(last_state.rot.inverse()* predicted_state.rot, last_state.rot.unrotate(predicted_state.p - last_state.p));
            lidar_measures.push_back(lidar_pR);
        }

        if(measurements.size() > WINDOW_SIZE)
        {
            measurements.erase(measurements.begin());
        }
        if(lidar_measures.size() > WINDOW_SIZE -1)
        {
            lidar_measures.erase(lidar_measures.begin());
        }

        std::cout << "Debug: traj_idx " << traj_idx << std::endl;

        gtsam::Vector4 input;
        if(traj_idx >= WINDOW_SIZE - 1)
        {
            for (int idx = 0; idx < OPT_LENS_TRAJ + WINDOW_SIZE; idx++)
            {
                if(idx >= WINDOW_SIZE - 1)
                {
                    DynamicsFactorTm dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1), V(idx + 1), S(idx + 1), dt, dynamics_noise);
                    graph.add(dynamics_factor);
                    
                    int future_idx = (idx - WINDOW_SIZE + 2);

                    gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + future_idx * dt)), circle_generator.pos(t0 + future_idx * dt));
                    
                    gtsam::Vector3 vel_idx = circle_generator.vel(t0 + future_idx * dt);
                    gtsam::Vector3 omega_idx = circle_generator.omega(t0 + future_idx * dt);

                    initial_value.insert(X(idx + 1), pose_idx);
                    initial_value.insert(V(idx + 1), vel_idx);
                    initial_value.insert(S(idx + 1), omega_idx);
                    
                    gtsam::Vector4 init_input = circle_generator.inputfm(t0 + future_idx * dt);
                    
                    if(idx != WINDOW_SIZE - 1)
                    {
                        BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                        graph.add(bet_FM_factor);
                    }
                    initial_value.insert(U(idx), init_input);
                    // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
                    
                    if(idx == OPT_LENS_TRAJ + WINDOW_SIZE - 1)
                    {
                        gtsam::Vector3 final_position_ref(CONTROL_P_FINAL_COV_X, CONTROL_P_FINAL_COV_Y, CONTROL_P_FINAL_COV_Z);
                        auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), final_position_ref).finished());   
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
                    }
                    else
                    {
                        gtsam::Vector3 _position_ref(CONTROL_P_COV_X, CONTROL_P_COV_Y, CONTROL_P_COV_Z);
                        auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), _position_ref).finished());
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                        // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
                    }
                }
                
                if (idx < WINDOW_SIZE)
                {
                    if(idx == WINDOW_SIZE - 1 )
                    {
                        graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), measurements[idx].v, vel_noise));
                        graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), measurements[idx].body_rate, omega_noise));
                        initial_value.insert(V(idx), measurements[idx].v);
                        initial_value.insert(S(idx), measurements[idx].body_rate);
                    }

                    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), gtsam::Pose3(measurements[idx].rot, measurements[idx].p), vicon_noise));
                    
                    if(idx > 0)
                    {
                        auto bet_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.003), Vector3::Constant(0.003)).finished());

                        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(idx-1), X(idx), lidar_measures[idx].compose(gtsam::Pose3(gtsam::Rot3::Expmap(lidar_noise_add.tail(3)),
                         gtsam::Vector3(lidar_noise_add.head(3)))), bet_noise));
                    }

                    initial_value.insert(X(idx), gtsam::Pose3(measurements[idx].rot, measurements[idx].p) );
                    
                }

            }

            std::cout << "###################### init optimizer ######################" << std::endl;
            LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

            std::cout << "###################### begin optimize ######################" << std::endl;
            Values result = optimizer.optimize();

            

            gtsam::Pose3   i_pose;
            gtsam::Vector3 vel;
            gtsam::Vector3 omega;
            
            state_trj.clear();
            opt_trj.clear();

            for (uint32_t ikey = 0; ikey < OPT_LENS_TRAJ + WINDOW_SIZE; ikey++)
            {
                    std::cout << red << "--------------------------------- TRAJECTORY CONTROL OPTIMIZATION: "  << ikey << " ----------------------------------" << def << std::endl;
                    i_pose = result.at<Pose3>(X(ikey));
                    std::cout << green << "OPT Translation: "
                            << i_pose.translation() << std::endl;
                    double t_ref = t0 + (ikey - WINDOW_SIZE + 1) * dt;

                    gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t_ref)), circle_generator.pos(t_ref));
                    std::cout << "REF Translation: "
                            << ref_pose.translation() << std::endl;

                    std::cout << "OPT    Rotation: "
                            << Rot3::Logmap(i_pose.rotation()).transpose() << std::endl;
                    std::cout << "REF    Rotation: "
                            << Rot3::Logmap(ref_pose.rotation()).transpose() << std::endl;

                    // vel = result.at<Vector3>(V(ikey));
                    // std::cout << "OPT         VEL: "
                    //         << vel.transpose() << std::endl;
                    // gtsam::Vector3 ref_vel = circle_generator.vel(t_ref); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                    // std::cout << "REF         VEL: "
                    //         << ref_vel.transpose() << std::endl;

                    // omega = result.at<Vector3>(S(ikey));
                    // std::cout << "OPT       OMEGA: "
                    //         << omega.transpose() << std::endl;
                    // gtsam::Vector3 ref_omega = circle_generator.omega(t_ref); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                    // std::cout << "REF       OMEGA: "
                    //         << ref_omega.transpose() << std::endl;

                    // if(ikey != OPT_LENS_TRAJ + WINDOW_SIZE - 1)
                    // {
                    //         input = result.at<gtsam::Vector4>(U(ikey + WINDOW_SIZE - 1));
                    //         std::cout << "OPT      INPUT: "
                    //                 << input.transpose() << std::endl;
                    //         std::cout << "REF      INPUT: "
                    //                 << circle_generator.inputfm(t_ref).transpose() << std::endl;
                    // }
                    State m_state;
                    m_state.p = i_pose.translation();
                    if(ikey < WINDOW_SIZE)
                    {
                        state_trj.push_back(m_state);
                    }
                    else
                    {
                        opt_trj.push_back(m_state);
                    }

                    std::cout << "state_trj size: " << state_trj.size() << std::endl;

            }

            input = result.at<gtsam::Vector4>(U(WINDOW_SIZE-1));
            predicted_state.thrust_torque = input;
            predicted_state.timestamp = t0 + dt;
            last_state = predicted_state;
            quadrotor.setState(predicted_state);

            input = result.at<gtsam::Vector4>(U(WINDOW_SIZE));
            quadrotor.stepODE(dt, input);  
        }
        else
        {
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

        std::cout << "predicted_pose: " << predicted_pose.translation() << std::endl;

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position     = circle_generator.pos(t0 + 1 * dt);
        gtsam::Vector3 tar_theta        = circle_generator.theta(t0 + 1 * dt);
        gtsam::Rot3    tar_rotation     = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel          = circle_generator.vel(t0 + 1 * dt);
        gtsam::Vector3 tar_omega        = circle_generator.omega(t0 + 1 * dt);
        gtsam::Vector4 ref_input        = circle_generator.inputfm(t0);

        gtsam::Vector3 err              = predicted_state.p - tar_position;
        gtsam::Vector3 pred_theta       = gtsam::Rot3::Logmap(predicted_state.rot);
        gtsam::Vector3 rot_err          = tar_rotation.rpy() - predicted_state.rot.rpy();
        gtsam::Vector4 actuator_outputs = quadrotor.CumputeRotorsVel();
        std::cout << "Before renderHistoryOpt: " << predicted_pose.translation() << std::endl;
        
        if(traj_idx < WINDOW_SIZE)
        {
            quadrotor.renderHistoryTrj();
        }
        else
        {
            quadrotor.renderHistoryOpt(opt_trj, err, landmarkk, vicon_measurement, rot_err, state_trj);
        }
        
        
        std::cout << "renderHistoryOpt: " << predicted_pose.translation() << std::endl;
        
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
