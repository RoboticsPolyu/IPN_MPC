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
using namespace UAV_Factor;

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
    YAML::Node FGO_config = YAML::LoadFile("../config/factor_graph.yaml");  
    double PRI_VICON_COV = FGO_config["PRI_VICON_COV"].as<double>();  
    double CONTROL_P_COV = FGO_config["CONTROL_P_COV"].as<double>();
    double CONTROL_P_FINAL_COV = FGO_config["CONTROL_P_FINAL_COV"].as<double>();
    double DYNAMIC_P_COV = FGO_config["DYNAMIC_P_COV"].as<double>(); 
    double CONTROL_R_COV = FGO_config["CONTROL_R_COV"].as<double>();
    uint16_t OPT_LENS_TRAJ = FGO_config["OPT_LENS_TRAJ"].as<uint16_t>();

    double PRIOR_U_F_COV = FGO_config["PRIOR_U_F_COV"].as<double>(); 
    double PRIOR_U_M1_COV = FGO_config["PRIOR_U_M1_COV"].as<double>(); 
    double PRIOR_U_M2_COV = FGO_config["PRIOR_U_M2_COV"].as<double>(); 
    double PRIOR_U_M3_COV = FGO_config["PRIOR_U_M3_COV"].as<double>(); 
    
    uint64_t SIM_STEPS = FGO_config["SIM_STEPS"].as<uint64_t>();

    YAML::Node quadrotor_config = YAML::LoadFile("../config/quadrotor.yaml");  
    double RADIUS = quadrotor_config["RADIUS"].as<double>();
    double LINEAR_VEL = quadrotor_config["LINEAR_VEL"].as<double>();
    double POS_MEAS_COV = quadrotor_config["POS_MEAS_COV"].as<double>();
    double POS_MEAS_MEAN = quadrotor_config["POS_MEAS_MEAN"].as<double>();

    double MAP_X = quadrotor_config["MAP_X"].as<double>();
    double MAP_Y = quadrotor_config["MAP_Y"].as<double>();
    double MAP_CENTER_X = quadrotor_config["MAP_CENTER_X"].as<double>();
    double MAP_CENTER_Y = quadrotor_config["MAP_CENTER_Y"].as<double>();
    double LIDAR_RANGE = quadrotor_config["LIDAR_RANGE"].as<double>();
    double LANDMARKS_SIZE = quadrotor_config["LANDMARKS_SIZE"].as<uint32_t>();

    double dt = 0.001f, radius = RADIUS, linear_vel = LINEAR_VEL;
    circle_generator circle_generator(radius, linear_vel, dt);

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations = 500;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(4, 1, 1, 1));
    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(PRIOR_U_F_COV, PRIOR_U_M1_COV, PRIOR_U_M2_COV, PRIOR_U_M3_COV));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(DYNAMIC_P_COV), Vector3::Constant(0.0005), Vector3::Constant(0.0005), Vector3::Constant(0.0005)).finished());
    
    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.005), Vector3::Constant(PRI_VICON_COV)).finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(.3, .3, .3));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(.3, .3, .3));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor;
    Quadrotor::State predicted_state;
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;
    std::normal_distribution<double> position_noise(POS_MEAS_MEAN, POS_MEAS_COV);
    
    Features landmarkk; 
    Landmarks env(MAP_X, MAP_Y, MAP_CENTER_X, MAP_CENTER_Y, LANDMARKS_SIZE);
    Lidar<Landmarks> lidar(LIDAR_RANGE);
    
    for(int traj_idx = 0; traj_idx < SIM_STEPS; traj_idx++)
    {
        double t0 = traj_idx* dt;

        if(traj_idx == 0)
        {
            predicted_state.x =  circle_generator.pos(t0);//  + gtsam::Vector3(0.05, 0, 0.10);
            predicted_state.rot = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            predicted_state.v = circle_generator.vel(t0);// gtsam::Vector3(0.05, 0, 0.10);
            predicted_state.omega = circle_generator.omega(t0);
            predicted_state.force_moment = circle_generator.inputfm(t0);
        }
        
        if(traj_idx == 1000)
        {
            predicted_state.x[0] = predicted_state.x[0] + 0.10f;
            predicted_state.x[2] = predicted_state.x[2] - 0.08f;
        }

        NonlinearFactorGraph graph;
        Values initial_value;
        graph.empty();

        for (int idx = 0; idx < OPT_LENS_TRAJ; idx++)
        {
            DynamicsFactorfm dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1), V(idx + 1), S(idx + 1), dt, dynamics_noise);
            graph.add(dynamics_factor);
            
            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + (idx + 1) * dt)), circle_generator.pos(t0 + (idx + 1) * dt));
            gtsam::Vector3 vel_idx = circle_generator.vel(t0 + (idx + 1) * dt);
            gtsam::Vector3 omega_idx = circle_generator.omega(t0 + (idx + 1) * dt);

            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);
            initial_value.insert(S(idx + 1), omega_idx);
            
            gtsam::Vector4 init_input = circle_generator.inputfm(t0 + idx * dt);
            if(idx != 0)
            {
                BetForceMoment bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                graph.add(bet_FM_factor);
            }
            initial_value.insert(U(idx), init_input);
            graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            
            if(idx == OPT_LENS_TRAJ - 1)
            {
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), Vector3::Constant(CONTROL_P_FINAL_COV)).finished());  
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
            }
            else
            {
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), Vector3::Constant(CONTROL_P_COV)).finished());  
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
            }

            if (idx == 0)
            {                
                gtsam::Vector3 pos_noise = gtsam::Vector3(position_noise(meas_x_gen), position_noise(meas_y_gen), position_noise(meas_z_gen));

                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), gtsam::Pose3(predicted_state.rot, predicted_state.x + pos_noise), vicon_noise));

                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), predicted_state.v, vel_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), predicted_state.omega, omega_noise));

                initial_value.insert(X(idx), gtsam::Pose3(predicted_state.rot, predicted_state.x + pos_noise));
                initial_value.insert(V(idx), predicted_state.v);
                initial_value.insert(S(idx), predicted_state.omega);
            }

        }

        std::cout << "###################### init optimizer ######################" << std::endl;
        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        std::cout << "###################### begin optimize ######################" << std::endl;
        Values result = optimizer.optimize();

        std::vector<Quadrotor::State> opt_trj;

        gtsam::Pose3 i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < OPT_LENS_TRAJ; ikey++)
        {
                std::cout << red << "--------------------------------- TRAJECTORY CONTROL OPTIMIZATION: "  << ikey << " ----------------------------------" << def << std::endl;
                i_pose = result.at<Pose3>(X(ikey));
                std::cout << green << "OPT Translation: "
                        << i_pose.translation() << std::endl;
                gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t0 + ikey * dt)), circle_generator.pos(t0 + ikey * dt));
                std::cout << "REF Translation: "
                        << ref_pose.translation() << std::endl;

                std::cout << "OPT    Rotation: "
                        << Rot3::Logmap(i_pose.rotation()).transpose() << std::endl;
                std::cout << "REF    Rotation: "
                        << Rot3::Logmap(ref_pose.rotation()).transpose() << std::endl;

                vel = result.at<Vector3>(V(ikey));
                std::cout << "OPT         VEL: "
                        << vel.transpose() << std::endl;
                gtsam::Vector3 ref_vel = circle_generator.vel(t0 + ikey * dt); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                std::cout << "REF         VEL: "
                        << ref_vel.transpose() << std::endl;

                omega = result.at<Vector3>(S(ikey));
                std::cout << "OPT       OMEGA: "
                        << omega.transpose() << std::endl;
                gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                std::cout << "REF       OMEGA: "
                        << ref_omega.transpose() << std::endl;

                if(ikey != OPT_LENS_TRAJ - 1)
                {
                        input = result.at<gtsam::Vector4>(U(ikey));
                        std::cout << "OPT      INPUT: "
                                << input.transpose() << std::endl;
                        std::cout << "REF      INPUT: "
                                << circle_generator.inputfm(t0 + ikey * dt).transpose() << std::endl;
                }
                Quadrotor::State m_state;
                m_state.x = i_pose.translation();
                opt_trj.push_back(m_state);

        }

        input = result.at<gtsam::Vector4>(U(0));
        predicted_state.force_moment = input;
        predicted_state.timestamp = t0 + dt;
        quadrotor.setState(predicted_state);

        input = result.at<gtsam::Vector4>(U(1));
        quadrotor.stepODE(dt, input);  

        predicted_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(predicted_state.rot, predicted_state.x);

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position = circle_generator.pos(t0 + 1 * dt);
        gtsam::Vector3 err = predicted_state.x - tar_position;

        quadrotor.render_history_opt(opt_trj, err, landmarkk);
    }

    while (true)
    {
        quadrotor.render_history_trj();
    }

    return 0;
}
