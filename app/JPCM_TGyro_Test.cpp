#include "color.h"
#include "env_sensors_sim/Landmarks.h"
#include "env_sensors_sim/Lidar.h"
#include "quadrotor_simulator/Dynamics_control_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"
#include "trajectory_generator/Trajectory_generator.h"
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

int main(void)
{
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier green(Color::FG_GREEN);
    clock_t start, end;

    // Configuration file 
    YAML::Node FGO_config        = YAML::LoadFile("../config/factor_graph_TGyro.yaml");  
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
    double CONTROL_R1_COV         = FGO_config["CONTROL_R1_COV"].as<double>();
    double CONTROL_R2_COV         = FGO_config["CONTROL_R2_COV"].as<double>();
    double CONTROL_R3_COV         = FGO_config["CONTROL_R3_COV"].as<double>();
    uint16_t OPT_LENS_TRAJ       = FGO_config["OPT_LENS_TRAJ"].as<uint16_t>();

    double PRIOR_U_F_COV         = FGO_config["PRIOR_U_F_COV"].as<double>(); 
    double PRIOR_U_M1_COV        = FGO_config["PRIOR_U_M1_COV"].as<double>(); 
    double PRIOR_U_M2_COV        = FGO_config["PRIOR_U_M2_COV"].as<double>(); 
    double PRIOR_U_M3_COV        = FGO_config["PRIOR_U_M3_COV"].as<double>(); 
    
    double INPUT_JERK_T          = FGO_config["INPUT_JERK_T"].as<double>(); 
    double INPUT_JERK_M          = FGO_config["INPUT_JERK_M"].as<double>(); 
    double INPUT_JERK_M3         = FGO_config["INPUT_JERK_M3"].as<double>(); 

    uint64_t SIM_STEPS           = FGO_config["SIM_STEPS"].as<uint64_t>();

    std::string LOG_NAME         = FGO_config["LOG_NAME"].as<std::string>();
    
    uint16_t WINDOW_SIZE         = FGO_config["WINDOW_SIZE"].as<uint16_t>();

    double high            = FGO_config["CLF_HIGH"].as<double>(); 
    double low             = FGO_config["CLF_LOW"].as<double>(); 
    double thr             = FGO_config["CLF_THR"].as<double>(); 
    double ghigh           = FGO_config["G_CLF_HIGH"].as<double>(); 
    double glow            = FGO_config["G_CLF_LOW"].as<double>(); 
    double gthr            = FGO_config["G_CLF_THR"].as<double>(); 
    double alpha           = FGO_config["CLF_ALPHA"].as<double>(); 

    YAML::Node quad_config = YAML::LoadFile("../config/quadrotor_TGyro.yaml"); 

    double RADIUS          = quad_config["RADIUS"].as<double>();
    double LINEAR_VEL      = quad_config["LINEAR_VEL"].as<double>();
    double POS_MEAS_COV    = quad_config["POS_MEAS_COV"].as<double>();
    double VEL_MEAS_COV    = quad_config["VEL_MEAS_COV"].as<double>();
    double ROT_MEAS_COV    = quad_config["ROT_MEAS_COV"].as<double>();
    double OME_MEAS_COV    = quad_config["OME_MEAS_COV"].as<double>();
    double POS_MEAS_MEAN   = quad_config["POS_MEAS_MEAN"].as<double>();
    bool   TEST_RECOVERY   = quad_config["TEST_RECOVERY"].as<bool>();

    double MAP_X           = quad_config["MAP_X"].as<double>();
    double MAP_Y           = quad_config["MAP_Y"].as<double>();
    double MAP_Z           = quad_config["MAP_Z"].as<double>();

    double MAP_CENTER_X    = quad_config["MAP_CENTER_X"].as<double>();
    double MAP_CENTER_Y    = quad_config["MAP_CENTER_Y"].as<double>();
    double MAP_CENTER_Z    = quad_config["MAP_CENTER_Z"].as<double>();
    double LIDAR_RANGE     = quad_config["LIDAR_RANGE"].as<double>();
    double LIDAR_RANGE_MIN = quad_config["LIDAR_RANGE_MIN"].as<double>();
    double LANDMARKS_SIZE  = quad_config["LANDMARKS_SIZE"].as<uint32_t>();
    
    double MOVE_X          = quad_config["MOVE_X"].as<double>();
    double MOVE_Y          = quad_config["MOVE_Y"].as<double>();
    double MOVE_Z          = quad_config["MOVE_Z"].as<double>();
    double mass            = 1.0f;
    gtsam::Vector3 drag_k(-0.0, -0., -0.);
    //gtsam::Vector3 drag_k(-0.20, -0.23, -0.31);

    std::ofstream JEC_log;
    std::string file_name = "../data/log/JPC_TGyro_";
    file_name.append(LOG_NAME);
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    double dt = 0.001f, radius = RADIUS, linear_vel = LINEAR_VEL;
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
    parameters.maxIterations    = 10;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(INPUT_JERK_T, INPUT_JERK_M, INPUT_JERK_M, INPUT_JERK_M3));
    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(PRIOR_U_F_COV, PRIOR_U_M1_COV, PRIOR_U_M2_COV, PRIOR_U_M3_COV));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(9) << Vector3::Constant(DYNAMIC_P_COV), Vector3::Constant(0.001), 
        Vector3::Constant(0.001)).finished());
    
    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(ROT_MEAS_COV), Vector3::Constant(PRI_VICON_COV)).finished());
    auto vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(PRI_VICON_VEL_COV, PRI_VICON_VEL_COV, PRI_VICON_VEL_COV));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(OME_MEAS_COV, OME_MEAS_COV, OME_MEAS_COV));
    
    auto point_obs_noise = noiseModel::Diagonal::Sigmas(Vector1(0.001));

    auto ref_predict_vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(CONTROL_V_COV, CONTROL_V_COV, CONTROL_V_COV));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(CONTROL_O_COV, CONTROL_O_COV, CONTROL_O_COV));

    dt = 0.01f; // Model predictive control duration

    Quadrotor quadrotor;
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

    std::normal_distribution<double> position_noise(POS_MEAS_MEAN, POS_MEAS_COV);
    std::normal_distribution<double> rot_noise(0, ROT_MEAS_COV);
    std::normal_distribution<double> velocity_noise(0, VEL_MEAS_COV);
    
    std::ofstream state_log;
    file_name = "../data/log/simulated_state.txt";
    state_log.open(file_name);

    std::ofstream pwm_log;
    file_name = "../data/log/simulated_state.txt";
    pwm_log.open(file_name);

    Features landmarkk; 
    Landmarks env(MAP_X, MAP_Y, MAP_Z, MAP_CENTER_X, MAP_CENTER_Y, MAP_CENTER_Z, LANDMARKS_SIZE);
    Lidar<Landmarks> lidar(LIDAR_RANGE, LIDAR_RANGE_MIN);
    gtsam::Vector3 vicon_measurement;
    gtsam::Vector4 rotor_input_bak;
    gtsam::Vector3 obs1(0, 0, 0);
    std::vector<gtsam::Vector3> obstacles;

    float obs1_radius = 0.20f, safe_d = 0.10f;

    for(int traj_idx = 0; traj_idx < SIM_STEPS; traj_idx++)
    {
        std::vector<State> opt_trj, ref_trj;
        obs1 = quadrotor.getObs1();
        // obstacles = quadrotor.getObstacles();
        // std::cout << " Obstacles size is " << obstacles.size() << std::endl;
        double t0 = traj_idx * dt;

        if(traj_idx == 0)
        {
            // predicted_state.p            = circle_generator.pos(t0) - gtsam::Vector3(0,0,1);
            // // gtsam::Vector3 rzyx(0, 0, 10.0/180.0*3.14159);
            // // gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(rzyx);
            // gtsam::Rot3 rot = gtsam::Rot3::identity();
            // predicted_state.rot          = rot; // gtsam::Rot3::Expmap(circle_generator.theta(t0));
            // predicted_state.v            = gtsam::Vector3::Zero();  // circle_generator.vel(t0);
            // predicted_state.body_rate    = gtsam::Vector3::Zero(); // circle_generator.omega(t0);
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

            predicted_state.p         = fig_gen.pos(0.0);
            gtsam::Rot3           rot = gtsam::Rot3::Expmap(fig_gen.theta(0.0)).toQuaternion();
            predicted_state.rot       = rot; 
            predicted_state.v         = fig_gen.vel(0.0); 
            predicted_state.body_rate = fig_gen.omega(0.0);
            // predicted_state.thrust_torque = circle_generator.inputfm(t0);

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
        auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
        ControlLimitTGyroFactor cntrolLimitTGyroFactor(U(0), clf_sigma, low, high, glow, ghigh, thr, gthr, alpha);
        graph.add(cntrolLimitTGyroFactor);

        for (int idx = 0; idx < OPT_LENS_TRAJ; idx++)
        {   
            DynamicsFactorTGyro dynamics_factor(X(idx), V(idx), U(idx), X(idx + 1), V(idx + 1), dt, mass, drag_k, dynamics_noise);
            graph.add(dynamics_factor);
            
            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(fig_gen.theta(t0 + (idx + 1) * dt)), fig_gen.pos(t0 + (idx + 1) * dt));
            
            gtsam::Vector3 vel_idx = fig_gen.vel(t0 + (idx + 1) * dt);
            float hvel = 0.50f;
            // gtsam::Pose3 pose_idx;
            // gtsam::Vector3 vel_idx;

            // if((t0 + float(idx+1)/100.0) < 4)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, 0.0 + (t0 + float(idx+1)/100.0)* hvel));
            //     vel_idx = gtsam::Vector3(0, 0, hvel);
            // }
            // else if((t0 + float(idx+1)/100.0) > 4 && (t0 + float(idx+1)/100.0) < 8)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.30, 0.40, 0.0 + (t0 + float(idx+1)/100.0)* hvel));
            //     vel_idx = gtsam::Vector3(0, 0, hvel);
            // }
            // else if((t0 + float(idx+1)/100.0) >= 8 && (t0 + float(idx+1)/100.0) < 12)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.30, 0.40, 0.0 + 8* hvel));
            //     vel_idx = gtsam::Vector3(0, 0, 0);
            // }
            // else if((t0 + float(idx+1)/100.0) >= 12 && (t0 + float(idx+1)/100.0) <= 16)
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.50, 0.70, 0.0 + 8* hvel - ((t0 + float(idx+1)/100.0) - 12)* hvel));
            //     vel_idx = gtsam::Vector3(0, 0, -hvel);
            // } 
            // else 
            // {
            //     pose_idx = gtsam::Pose3(gtsam::Rot3(), gtsam::Vector3(0.50, 0.70, 0.0 + 8* hvel - 4* hvel));
            //     vel_idx = gtsam::Vector3(0, 0, 0);    
            // }
            
            gtsam::Vector3 omega_idx = fig_gen.omega(t0 + (idx + 1) * dt);
            
            State ref_state;
            ref_state.p   = pose_idx.translation();
            ref_state.rot = pose_idx.rotation();
            ref_state.v   = vel_idx;
            ref_trj.push_back(ref_state);

            gtsam::Vector4 init_input(10, 0, 0, 0);
            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);
            initial_value.insert(U(idx),     init_input);

            // gtsam::Vector4 init_input = circle_generator.inputfm(t0 + idx * dt);
            if(idx != 0)
            {
                BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
                graph.add(bet_FM_factor);
            }
            // initial_value.insert(U(idx), init_input); //
            // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            gtsam::Vector3 control_r_cov(CONTROL_R1_COV, CONTROL_R2_COV, CONTROL_R3_COV);
            if(idx == OPT_LENS_TRAJ - 1)
            {   
                gtsam::Vector3 final_position_ref(CONTROL_P_FINAL_COV_X, CONTROL_P_FINAL_COV_Y, CONTROL_P_FINAL_COV_Z);
                // auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), final_position_ref).finished());   
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, final_position_ref).finished()); 
                
                // float d2 = std::sqrt((pose_idx.translation() - obs1).transpose()* (pose_idx.translation() - obs1));
                
                // if(d2 < obs1_radius)
                // {
                //     float scale = obs1_radius / d2; 
                //     pose_idx = gtsam::Pose3(pose_idx.rotation(), (pose_idx.translation() - obs1)* scale + obs1);
                //     graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                //     graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                // }
                // else
                // {
                    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1),   pose_idx, ref_predict_pose_noise));
                    graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,  ref_predict_vel_noise));
                // }

                    // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
                    // auto correction_noise = noiseModel::Isotropic::Sigma(3, CONTROL_P_FINAL_COV_X);
                    // gtsam::GPSFactor gps_factor(X(idx+1),
                    //            Point3(pose_idx.translation()[0],   // N,
                    //                   pose_idx.translation()[1],   // E,
                    //                   pose_idx.translation()[2]),  // D,
                    //            correction_noise);
                    // graph.add(gps_factor);
                // for(uint16_t obsi = 0; obsi < obstacles.size(); obsi++)
                // {
                //     obs1 = obstacles[obsi];
                //     graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d, point_obs_noise));
                // }
            }
            else
            {
                gtsam::Vector3 _position_ref(CONTROL_P_COV_X, CONTROL_P_COV_Y, CONTROL_P_COV_Z);
                // auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(CONTROL_R_COV), _position_ref).finished());
                auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, _position_ref).finished());
                


                    // float d2 = std::sqrt((pose_idx.translation() - obs1).transpose()* (pose_idx.translation() - obs1));
                    // if(d2 < obs1_radius)
                    // {
                    //     float scale = obs1_radius / d2; 
                    //     pose_idx = gtsam::Pose3(pose_idx.rotation(), (pose_idx.translation() - obs1)* scale + obs1);
                    // }
                    // if((pose_idx.translation() - obs1).transpose()* (pose_idx.translation() - obs1) >= 0.50 * 0.50)
                    // {
                    //     graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                    //     graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                    // }
                //   else
                //    {
                        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1),   pose_idx, ref_predict_pose_noise));
                        graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx,  ref_predict_vel_noise));

                //    }
                    // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));
                    // auto correction_noise = noiseModel::Isotropic::Sigma(3, CONTROL_P_COV_X);
                    // gtsam::GPSFactor gps_factor(X(idx + 1),
                    //            Point3(pose_idx.translation()[0],   // N,
                    //                   pose_idx.translation()[1],   // E,
                    //                   pose_idx.translation()[2]),  // D,
                    //            correction_noise);
                    // graph.add(gps_factor);

                // for(uint16_t obsi=0; obsi < obstacles.size(); obsi++)
                // {
                //     obs1 = obstacles[obsi];
                //     graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d, point_obs_noise));
                // }
            }

            // for(uint16_t obsi = 0; obsi < obstacles.size(); obsi++)
            // {
            //     obs1 = obstacles[obsi];
            //     graph.add(PointObsFactor(X(idx+1), obs1, obs1_radius + safe_d, point_obs_noise));
            // }

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
                // graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), predicted_state.body_rate, omega_noise));
                
                std::cout << "pose noise: " << pos_noise.transpose() << std::endl;
                
                initial_value.insert(X(idx), gtsam::Pose3(predicted_state.rot, vicon_measurement));
                initial_value.insert(V(idx), vel_add);
                // initial_value.insert(S(idx), predicted_state.body_rate);
            }

        }

        
        std::cout << "###################### init optimizer ######################" << std::endl;
        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);
        BatchFixedLagSmoother smoother(7.0, LevenbergMarquardtParams());
        
        std::cout << "###################### begin optimize ######################" << std::endl;
        start = clock();
        Values result = optimizer.optimize();
	    end = clock();
        float opt_cost = (double)(end - start)/CLOCKS_PER_SEC;
	    std::cout << " ---------- Optimize Time " << opt_cost << endl;

        gtsam::Pose3   i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < OPT_LENS_TRAJ; ikey++)
        {
                std::cout << red << "--------------------------------- TRAJECTORY CONTROL OPTIMIZATION: "  << ikey << " ----------------------------------" << def << std::endl;
                i_pose = result.at<Pose3>(X(ikey));
                vel = result.at<Vector3>(V(ikey));
                // omega = result.at<Vector3>(S(ikey));
                // gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt);
                // gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t0 + ikey * dt)), circle_generator.pos(t0 + ikey * dt));
                // gtsam::Vector3 ref_vel = circle_generator.vel(t0 + ikey * dt);
                
                std::cout << green << "OPT Translation: "
                        << i_pose.translation().transpose() << std::endl;
                // std::cout << "REF Translation: "
                //         << ref_pose.translation() << std::endl;

                std::cout << "OPT    Rotation: "
                        << Rot3::Logmap(i_pose.rotation()).transpose() << std::endl;
                // std::cout << "REF    Rotation: "
                //         << Rot3::Logmap(ref_pose.rotation()).transpose() << std::endl;

                std::cout << "OPT         VEL: "
                        << vel.transpose() << std::endl;
                 //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
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
                        std::cout << "OPT      INPUT: "
                                << input.transpose() << std::endl;
                        // std::cout << "REF      INPUT: "
                        //         << circle_generator.inputfm(t0 + ikey * dt).transpose() << std::endl;
                }
                State m_state;
                m_state.p = i_pose.translation();
                opt_trj.push_back(m_state);

        }

        input = result.at<gtsam::Vector4>(U(0));

        std::cout << " ------ " << input.transpose() << std::endl;
        
        /* Simulator */
        State est_state = quadrotor.getState();
        gtsam::Vector3 drag_force = est_state.rot.matrix() * Eigen::Matrix3d(drag_k.asDiagonal()) * est_state.rot.matrix().transpose() * est_state.v;
        
        float g_ = 9.81f;
        std::normal_distribution<double> thrust_noise(0, 0.2);
        std::normal_distribution<double> gyro_noise(0, 0.02);
        std::default_random_engine generator_;
        double at_noise = thrust_noise(generator_);
        double wx_noise = gyro_noise(generator_);
        double wy_noise = gyro_noise(generator_);
        double wz_noise = gyro_noise(generator_);

        gtsam::Vector3 v_dot = - gtsam::Vector3(0, 0, g_) 
                                + est_state.rot.rotate(gtsam::Vector3(0, 0, (input[0] + at_noise) / mass))
                                + drag_force / mass;

        gtsam::Vector3 p_dot = est_state.v;

        Eigen::Matrix3d r_dot = est_state.rot.matrix() * gtsam::skewSymmetric(est_state.body_rate);

        est_state.p = est_state.p + p_dot * dt; 
        est_state.v = est_state.v + v_dot * dt;
        gtsam::Vector3 body_rate = gtsam::Vector3(input[1], input[2], input[3]) + gtsam::Vector3(wx_noise, wy_noise, wz_noise);
        est_state.rot = est_state.rot* gtsam::Rot3::Expmap(body_rate* dt);

        quadrotor.setState(est_state);

        predicted_state = quadrotor.getState();
        gtsam::Pose3 predicted_pose = gtsam::Pose3(predicted_state.rot, predicted_state.p);

        landmarkk = lidar.Measurement(env, predicted_pose);
        gtsam::Vector3 tar_position     = fig_gen.pos(t0 + 1 * dt);
        gtsam::Vector3 tar_theta        = fig_gen.theta(t0 + 1 * dt);
        gtsam::Rot3    tar_rotation     = gtsam::Rot3::Expmap(tar_theta);
        gtsam::Vector3 tar_vel          = fig_gen.vel(t0 + 1 * dt);
        gtsam::Vector3 tar_omega        = fig_gen.omega(t0 + 1 * dt);
        gtsam::Vector4 ref_input        = fig_gen.inputfm(t0);

        gtsam::Vector3 pos_err          = predicted_state.p - tar_position;
        gtsam::Vector3 pred_theta       = gtsam::Rot3::Logmap(predicted_state.rot);
        gtsam::Vector3 rot_err          = tar_rotation.rpy() - predicted_state.rot.rpy();

        std::cout << "tar rot rpy: " << tar_rotation.rpy().transpose() << std::endl;
        std::cout << "predicted_state rot rpy: " << predicted_state.rot.rpy().transpose() << std::endl;
        // actuator_outputs = quadrotor.CumputeRotorsVel();
        
        // quadrotor.renderHistoryOpt(opt_trj, err, landmarkk, vicon_measurement, rot_err);
        quadrotor.renderHistoryOpt(opt_trj, pos_err, boost::none, vicon_measurement, rot_err, ref_trj, opt_cost);

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
            << ref_input[0] << " " << ref_input[1] << " " << ref_input[2] << " " << ref_input[3] << " "
            << opt_cost << " "
            << std::endl;
    }

    while (true)
    {
        quadrotor.renderHistoryTrj();
    }

    return 0;
}
