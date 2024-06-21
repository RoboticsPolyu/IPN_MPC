#include "calibration/Calibration_factor.h"
#include "quadrotor_simulator/Dynamics_params.h"


#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

using namespace gtsam;
using namespace std;
using namespace UAVFactor;


using symbol_shorthand::K; // kf
using symbol_shorthand::J; // inertial of moments
using symbol_shorthand::M; // km
using symbol_shorthand::P; // position of rotor
using symbol_shorthand::R; // rotation of gravity
using symbol_shorthand::S; // angular speed
using symbol_shorthand::T; // thrust and moments
using symbol_shorthand::U; // actuator_input
using symbol_shorthand::V; // velocity
using symbol_shorthand::X; // pose
using symbol_shorthand::D; // bTm
using symbol_shorthand::H;
using symbol_shorthand::A;
using symbol_shorthand::B;


typedef struct State
{
    int            id;
    double         timestamp;
    gtsam::Pose3   pose;
    gtsam::Vector3 vel;
    gtsam::Vector3 body_rate;
    gtsam::Vector4 actuator_output;

} State;

typedef struct Actuator_control
{
    int            id;
    double         timestamp;
    gtsam::Vector4 actuator_output;

} Actuator_control;

// Pose slerp interpolation
Pose3 interpolateRt(const::Pose3& T_l, const Pose3& T, double t) 
{
    return Pose3(gtsam::interpolate<Rot3>(T_l.rotation(), T.rotation(), t), gtsam::interpolate<Point3>(T_l.translation(), T.translation(), t));
}

int main(void)
{
    DynamicsParams quad_params;
    double      dt         = 0.0025f;
    quad_params.mass       = 0.772f;
    quad_params.Ixx        = 2.5 * 1e-3;
    quad_params.Iyy        = 2.1 * 1e-3;
    quad_params.Izz        = 4.3 * 1e-3;
    quad_params.k_f        = 1e-6; 
    quad_params.k_m        = 0.01;
    quad_params.arm_length = 0.13f;


    // Configuration file 
    YAML::Node CAL_config = YAML::LoadFile("../config/dynamics_NeuroBEM.yaml");  
    double     rotor_px   = CAL_config["ROTOR_P_X"].as<double>();
    double     rotor_py   = CAL_config["ROTOR_P_Y"].as<double>();
    uint16_t DATASET_S    = CAL_config["DATASET_S"].as<uint16_t>();
    uint16_t DATASET_LENS = CAL_config["DATASET_L"].as<uint16_t>();
    double p_thrust_sigma = CAL_config["P_T_SIGMA"].as<double>();
    double unmodel_thrust_sigma = CAL_config["U_T_SIGMA"].as<double>();
    std::string file_path = CAL_config["ROOT_PATH"].as<std::string>();
    bool ENABLE_COG       = CAL_config["ENABLE_COG"].as<bool>();
    bool ENABLE_VISCOUS   = CAL_config["ENABLE_VISCOUS"].as<bool>();
    bool enable_inertia   = CAL_config["ENABLE_I"].as<bool>();
    bool enable_drag      = CAL_config["ENABLE_DRAG"].as<bool>();
    double expan_pos_sigma = CAL_config["EXPAN_P_SIGMA"].as<double>();
    uint16_t batch_size   = CAL_config["BATCH_SIZE"].as<uint16_t>();

    gtsam::Vector3 thrust_sigma(unmodel_thrust_sigma, unmodel_thrust_sigma, 2* p_thrust_sigma);
    // sigma = (rotor_p_x* 2* P_thrust_single_rotor, rotor_p_x* 2* P_thrust_single_rotor, k_m * 2 * P_thrust_single_rotor) 
    gtsam::Vector3 moments_sigma(p_thrust_sigma * 2 * rotor_py, p_thrust_sigma * 2 * rotor_px, 0.01f * 2 * p_thrust_sigma);
    auto vel_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto ang_s_noise  = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));

    auto dyn_noise    = noiseModel::Diagonal::Sigmas((Vector(12) << thrust_sigma * 0.5f * dt * dt, Vector3::Constant(0.0005 * 0.01), 
                                                                   thrust_sigma * dt, moments_sigma * dt).finished());
    
    auto vicon_noise  = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(expan_pos_sigma > 0.001 ? expan_pos_sigma: 0.001f), Vector3::Constant(0.001)).finished());

    auto im_noise     = noiseModel::Diagonal::Sigmas(Vector3(0.000001, 0.000001, 0.000001));
    auto rp_noise     = noiseModel::Diagonal::Sigmas(Vector3(0.00001, 0.00001, 0.00001));
    auto gr_noise     = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto km_noise     = noiseModel::Diagonal::Sigmas(Vector1(0.00001));
    auto bm_nosie     = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.00001), Vector3::Constant(0.00001)).finished());

    std::ofstream calib_log;
    std::string file_name = "../data/calib_";
    file_name.append("debug");
    file_name.append("_log.txt");
    calib_log.open(file_name);

    std::ofstream calib_params_log;
    file_name = "../data/batch_calib_";
    file_name.append("params");
    file_name.append("_log.txt");
    calib_params_log.open(file_name);


    std::vector<State>   Interp_states;
    std::vector<Actuator_control> Uav_pwms;

    std::ifstream NeuroBEM_file;
    std::string NeuroBEM_path = "/Users/ypwen/IPN/IPN_MPC/data/NeuroBEM/test1/neuro_bem.txt";
    NeuroBEM_file.open(NeuroBEM_path);

    double gt_t, gt_qw, gt_qx, gt_qy, gt_qz, gt_x, gt_y, gt_z, pwm_t, pwm1, pwm2, pwm3, pwm4, gt_vx, gt_vy, gt_vz, ang_vel_x, ang_vel_y, ang_vel_z, vel_x, vel_y, vel_z;

    // double t	, ang_acc_x , ang_acc_y	, ang_acc_z , ang_vel_x , ang_vel_y ,	ang_vel_z ,	quat_x , quat_y	, quat_z , quat_w , acc_x , acc_y , acc_z , vel_x , vel_y , vel_z , pos_x , pos_y , pos_z , mot1 , mot2 , mot3 , mot4 , dmot1	, dmot2 , dmot3 , dmot4 , vbat;

    // while (NeuroBEM_file >> t	>> ang_acc_x >> ang_acc_y	>> ang_acc_z >> ang_vel_x	>> ang_vel_y >>	ang_vel_z >>	quat_x >> quat_y	>> quat_z >> quat_w >> acc_x >> acc_y >> acc_z >> vel_x >> vel_y >> vel_z >> pos_x >> pos_y >> pos_z >> mot1 >> mot2 >> mot3 >> mot4 >> dmot1	>> dmot2 >> dmot3 >> dmot4 >> vbat)
    while(NeuroBEM_file >> gt_t >> gt_qw >> gt_qx >> gt_qy >> gt_qz >> gt_vx >> gt_vy >> gt_vz >> gt_x >> gt_y >> gt_z >> pwm1 >> pwm2 >> pwm3 >> pwm4 >> ang_vel_x >> ang_vel_y >> ang_vel_z >> vel_x >> vel_y >> vel_z)
    {
        State _state;
        _state.timestamp   = gt_t;
        _state.pose        = gtsam::Pose3(gtsam::Quaternion(gt_qw, gt_qx, gt_qy, gt_qz), gtsam::Vector3(gt_x, gt_y, gt_z));
        _state.vel         = gtsam::Vector3(vel_x, vel_y, vel_z);
        _state.body_rate       = gtsam::Vector3(ang_vel_x, ang_vel_y, ang_vel_z);
        _state.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);

        // std::cout << "_state.actuator_output: " << _state.actuator_output.transpose() << "\n";
        Interp_states.push_back(_state);

        Actuator_control _uav_pwm;
        _uav_pwm.timestamp       = gt_t;
        _uav_pwm.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);
        Uav_pwms.push_back(_uav_pwm);
        
    }

    std::cout << "Data: " << Interp_states.size() << std::endl;
    // for(int index = 1; index < Interp_states.size(); index++)
    // {
    //     gtsam::Vector3 vel    = (Interp_states[index-1].pose.translation() - Interp_states[index].pose.translation())/dt;
    //     // gtsam::Vector3 ag_vel = gtsam::Rot3::Logmap(Interp_states[index-1].pose.rotation().between(Interp_states[index].pose.rotation()))/dt;

    //     Interp_states[index].vel = vel;
    //     // Interp_states[index].omega = ag_vel;

    // }

    // Define the smoother lag (in seconds)
    double lag = 2.0;

    // Create a fixed lag smoother
    // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
    BatchFixedLagSmoother smootherBatch(lag);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
    parameters.relinearizeSkip = 1; // Relinearize every time
    IncrementalFixedLagSmoother smootherISAM2(lag, parameters);

    NonlinearFactorGraph dyn_factor_graph;
    Values               initial_value_dyn;
    Values               result;

    FixedLagSmoother::KeyTimestampMap newTimestamps;

    dyn_factor_graph.empty();

    std::normal_distribution<double> pos_noise(0, expan_pos_sigma);
    std::default_random_engine meas_x_gen;

    gtsam::Vector3  IM = gtsam::Vector3(0);
    gtsam::Rot3    rot = gtsam::Rot3::identity();
    gtsam::Vector3   p = gtsam::Vector3(0);
    double          kf = 2.0e-06;
    double          km = 0.005; 
    gtsam::Pose3   bTm = gtsam::Pose3::identity(); 
    gtsam::Vector3  dk = gtsam::Vector3(0.0, 0.0, 0.0);
    gtsam::Vector3  ak = gtsam::Vector3(0.0, 0.0, 1.0);
    gtsam::Vector3  bk = gtsam::Vector3(0);

    gtsam::Vector3 rotor_p(rotor_px, rotor_py, 0);
    gtsam::Vector3 drag_k(0.000f, 0.000f, 0.000f);
    gtsam::Vector3 A_k(0.000f, 0.000f, 0.000f);
    gtsam::Vector3 B_k(0.000f, 0.000f, 0.000f);

    // newTimestamps[J(0)] = 0.0;
    // newTimestamps[D(0)] = 0.0;
    // newTimestamps[P(0)] = 0.0;
    // newTimestamps[M(0)] = 0.0;
    // newTimestamps[H(0)] = 0.0;
    // newTimestamps[A(0)] = 0.0;
    // newTimestamps[B(0)] = 0.0;

    initial_value_dyn.insert(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz) );
    initial_value_dyn.insert(R(0), rot);
    initial_value_dyn.insert(K(0), kf);
    initial_value_dyn.insert(M(0), km);
    initial_value_dyn.insert(D(0), bTm);

    initial_value_dyn.insert(P(0), rotor_p);

    initial_value_dyn.insert(H(0), dk);
    initial_value_dyn.insert(A(0), ak);
    initial_value_dyn.insert(B(0), bk);

    
    if(!enable_inertia)
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz), im_noise));
    }

    dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(D(0), gtsam::Pose3::identity(), bm_nosie));

    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Rot3>(R(0), gtsam::Rot3::identity(), gr_noise));
    dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(P(0), rotor_p, rp_noise)); 
    dyn_factor_graph.add(gtsam::PriorFactor<double>(M(0), 0.001f, km_noise));

    if(!enable_drag)
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(H(0), drag_k, im_noise)); 
    }
    if(!ENABLE_COG)
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(A(0), A_k, im_noise)); 
    }

    if(!ENABLE_VISCOUS)   
    {
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(B(0), B_k, im_noise)); 
    }

    for(uint32_t idx = DATASET_S; idx < DATASET_LENS; idx++)
    {
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;

        gtsam::Pose3 posej = Interp_states.at(idx).pose* 
            gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Vector3(pos_noise(meas_x_gen), pos_noise(meas_x_gen), pos_noise(meas_x_gen)));

        newTimestamps[X(idx)] = (idx - DATASET_S) * dt;
        newTimestamps[V(idx)] = (idx - DATASET_S) * dt;
        newTimestamps[S(idx)] = (idx - DATASET_S) * dt;

        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), Interp_states.at(idx).pose, vicon_noise));
        // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), Interp_states.at(idx).vel, vel_noise));
        // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), Interp_states.at(idx).body_rate, ang_s_noise));

        initial_value_dyn.insert(X(idx), Interp_states.at(idx).pose);
        initial_value_dyn.insert(V(idx), Interp_states.at(idx).vel);
        initial_value_dyn.insert(S(idx), Interp_states.at(idx).body_rate);

        // if( idx == DATASET_LENS-1)
        // {
        //     gtsam::Pose3 posej = Interp_states.at(idx+1).pose* 
        //         gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Vector3(pos_noise(meas_x_gen), pos_noise(meas_x_gen), pos_noise(meas_x_gen)));
        //     dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx+1), posej, vicon_noise));
        //     // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), Interp_states.at(idx+1).vel, vel_noise));
        //     // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), Interp_states.at(idx+1).body_rate, ang_s_noise));

        //     initial_value_dyn.insert(X(idx+1), Interp_states.at(idx+1).pose);
        //     initial_value_dyn.insert(V(idx+1), Interp_states.at(idx+1).vel);
        //     initial_value_dyn.insert(S(idx+1), Interp_states.at(idx+1).body_rate);
        // }
        
        // pose, velocity, angular speed, pose, velocity, angular speed, inertial of moments, rot of g, position of rotot, kf, km        
        DynamcisCaliFactor_RS_AB dynamicsCalibFactor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), D(0), H(0), A(0), B(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);
        dyn_factor_graph.add(dynamicsCalibFactor);

        if(idx % batch_size == 0 && idx != DATASET_S)
        {
            gtsam::Pose3 posej = Interp_states.at(idx+1).pose* 
              gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Vector3(pos_noise(meas_x_gen), pos_noise(meas_x_gen), pos_noise(meas_x_gen)));
            dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx+1), posej, vicon_noise));
            // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), Interp_states.at(idx+1).vel, vel_noise));
            // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), Interp_states.at(idx+1).body_rate, ang_s_noise));

            newTimestamps[X(idx+1)] = (idx - DATASET_S) * dt + dt;
            newTimestamps[V(idx+1)] = (idx - DATASET_S) * dt + dt;
            newTimestamps[S(idx+1)] = (idx - DATASET_S) * dt + dt;

            initial_value_dyn.insert(X(idx+1), Interp_states.at(idx+1).pose);
            initial_value_dyn.insert(V(idx+1), Interp_states.at(idx+1).vel);
            initial_value_dyn.insert(S(idx+1), Interp_states.at(idx+1).body_rate);

            idx++;

            gtsam::LevenbergMarquardtParams parameters;
            parameters.absoluteErrorTol = 1e-8;
            parameters.relativeErrorTol = 1e-8;
            parameters.maxIterations    = 500;
            parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
            parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
            std::cout << "###################### init contoller optimizer ######################" << std::endl;
            LevenbergMarquardtOptimizer optimizer(dyn_factor_graph, initial_value_dyn, parameters);

            // std::cout << "###################### begin optimize ######################" << std::endl;
            // smootherBatch.update(dyn_factor_graph, initial_value_dyn, newTimestamps);
            // smootherBatch.calculateEstimate<Pose2>(currentKey).print("Batch Estimate:");
            // result = smootherBatch.calculateEstimate();

            // smootherISAM2.update(dyn_factor_graph, initial_value_dyn, newTimestamps);
            // for(size_t i = 1; i < 2; ++i) { // Optionally perform multiple iSAM2 iterations
            //     smootherISAM2.update();
            // }

            std::cout << "###################### begin optimize ######################" << std::endl;
            result = optimizer.optimize();
            // result = smootherISAM2.calculateEstimate();

            IM  = result.at<gtsam::Vector3>(J(0));
            rot = result.at<gtsam::Rot3>(R(0));
            p   = result.at<gtsam::Vector3>(P(0));
            kf  = result.at<double>(K(0));
            km  = result.at<double>(M(0)); 
            bTm = result.at<gtsam::Pose3>(D(0)); 
            dk  = result.at<gtsam::Vector3>(H(0));
            ak  = result.at<gtsam::Vector3>(A(0));
            bk  = result.at<gtsam::Vector3>(B(0));

            calib_params_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) << " "
              << kf << " " << km << " " 
              << dk.x() << " " << dk.y() << " " << dk.z() << " " 
              << ak.x() << " " << ak.y() << " " << ak.z() << " " 
              << bk.x() << " " << bk.y() << " " << bk.z() << " "
              << rot.rpy().x() << " " << rot.rpy().y() << " " << rot.rpy().z() << " "
              << IM.x() << " " << IM.y() << " " << IM.z() << " " 
              << std::endl;

            // dyn_factor_graph.resize(0);
            // initial_value_dyn.clear();
            // newTimestamps.clear();

            DynamcisCaliFactor_RS_AB dynamicsCalibFactor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), 
                J(0), R(0), P(0), K(0), M(0), D(0), H(0), A(0), B(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);
            dyn_factor_graph.add(dynamicsCalibFactor);
        }
    }
    // print error
    
    // for(uint32_t idx = DATASET_S; idx < DATASET_LENS - 1; idx++)
    // {
    //     gtsam::Pose3    pi = result.at<gtsam::Pose3>(X(idx));
    //     gtsam::Vector3  vi = result.at<gtsam::Vector3>(V(idx));
    //     gtsam::Vector3  vj = result.at<gtsam::Vector3>(V(idx+1));
    //     gtsam::Vector3  oi = result.at<gtsam::Vector3>(S(idx));
    //     gtsam::Vector3  oj = result.at<gtsam::Vector3>(S(idx+1));

    //     DynamcisCaliFactor_RS_AB dyn_err(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), D(0), H(0), A(0), B(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);

    //     gtsam::Pose3          pose = result.at<gtsam::Pose3>(X(idx));
    //     gtsam::Pose3          pose_j = result.at<gtsam::Pose3>(X(idx+1));

    //     // gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, vi, oi, Interp_states.at(idx+1).pose, vj, oj, IM, rot, p, kf, km, bTm, dk, ak, bk);
    //     gtsam::Vector12 dyn_e = dyn_err.evaluateError(pose, pose.rotation().rotate(Interp_states.at(idx).vel), Interp_states.at(idx).body_rate, 
    //                             pose_j, pose_j.rotation().rotate(Interp_states.at(idx+1).vel), Interp_states.at(idx+1).body_rate, 
    //                             IM, rot, p, kf, km, bTm, dk, ak, bk);
    //     gtsam::Vector4 rpm_square = Interp_states.at(idx).actuator_output.cwiseAbs2();
    //     gtsam::Vector6 thrust_torque = dyn_err.Thrust_Torque(rpm_square, kf, km, p);

    //     gtsam::Vector3 dyn_pos_err = pose.rotation() * gtsam::Vector3(dyn_e(0), dyn_e(1), dyn_e(2)) * quad_params.mass;

    //     // gtsam::Vector3 dyn_vel_err = pose.rotation() * gtsam::Vector3(dyn_e(3), dyn_e(4), dyn_e(5)) * quad_params.mass;

    //     gtsam::Matrix3 drag_matrix;
    //     drag_matrix.setZero();
    //     drag_matrix.diagonal() << dk;  

    //     gtsam::Vector3 drag_force = quad_params.mass * drag_matrix * pi.rotation().unrotate(vi);

    //     gtsam::Vector3 vel_body = pose.rotation().unrotate(vi);

    //     float hor_thrust = ak.z() * (vi.x()* vi.x() + vi.y()* vi.y());


    //     calib_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) 
    //     << " " << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) 
    //     << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) 
    //     << " " << thrust_torque(0)<< " " << thrust_torque(1) << " " << thrust_torque(2) << " " << thrust_torque(3) << " " << thrust_torque(4) << " " << thrust_torque(5) 
    //     << " " << vel_body(0) << " " << vel_body(1) << " " << vel_body(2) 
    //     << " " << Interp_states.at(idx).body_rate.x() << " " << Interp_states.at(idx).body_rate.y() << " " << Interp_states.at(idx).body_rate.z() 
    //     << " " << drag_force(0) << " " << drag_force(1) << " " << drag_force(2) << " " << hor_thrust << std::endl;
    //     // << " " << Interp_states.at(idx).vel.x() << " " << Interp_states.at(idx).vel.y() << " " << Interp_states.at(idx).vel.z() << std::endl;
    // }

     
    for(uint32_t idx = DATASET_S; idx < DATASET_LENS - 1; idx++)
    {

        DynamcisCaliFactor_RS_AB dyn_err(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), D(0), H(0), A(0), B(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);

        gtsam::Pose3          pose   = Interp_states.at(idx).pose;
        gtsam::Pose3          pose_j = Interp_states.at(idx+1).pose;
        gtsam::Vector3        vi     = Interp_states.at(idx).vel;

        // gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, vi, oi, Interp_states.at(idx+1).pose, vj, oj, IM, rot, p, kf, km, bTm, dk, ak, bk);
        gtsam::Vector12 dyn_e = dyn_err.evaluateError(pose, pose.rotation().rotate(Interp_states.at(idx).vel), Interp_states.at(idx).body_rate, 
                                pose_j, pose_j.rotation().rotate(Interp_states.at(idx+1).vel), Interp_states.at(idx+1).body_rate, 
                                IM, rot, p, kf, km, bTm, dk, ak, bk);
        gtsam::Vector4 rpm_square = Interp_states.at(idx).actuator_output.cwiseAbs2();
        gtsam::Vector6 thrust_torque = dyn_err.Thrust_Torque(rpm_square, kf, km, p);

        gtsam::Vector3 dyn_pos_err = pose.rotation() * gtsam::Vector3(dyn_e(0), dyn_e(1), dyn_e(2)) * quad_params.mass;

        // gtsam::Vector3 dyn_vel_err = pose.rotation() * gtsam::Vector3(dyn_e(3), dyn_e(4), dyn_e(5)) * quad_params.mass;

        gtsam::Matrix3 drag_matrix;
        drag_matrix.setZero();
        drag_matrix.diagonal() << dk;  

        gtsam::Vector3 drag_force = quad_params.mass * drag_matrix * pose.rotation().unrotate(vi);

        gtsam::Vector3 vel_body = pose.rotation().unrotate(vi);

        float hor_thrust = ak.z() * (vi.x()* vi.x() + vi.y()* vi.y());


        calib_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) 
        << " " << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) 
        << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) 
        << " " << thrust_torque(0)<< " " << thrust_torque(1) << " " << thrust_torque(2) << " " << thrust_torque(3) << " " << thrust_torque(4) << " " << thrust_torque(5) 
        << " " << vel_body(0) << " " << vel_body(1) << " " << vel_body(2) 
        << " " << Interp_states.at(idx).body_rate.x() << " " << Interp_states.at(idx).body_rate.y() << " " << Interp_states.at(idx).body_rate.z() 
        << " " << drag_force(0) << " " << drag_force(1) << " " << drag_force(2) << " " << hor_thrust << std::endl;
        // << " " << Interp_states.at(idx).vel.x() << " " << Interp_states.at(idx).vel.y() << " " << Interp_states.at(idx).vel.z() << std::endl;
    }

    std::cout << "Inertial of moment:" << IM.transpose() << "\n";
    std::cout << "Gravity Rot:     " << rot.rpy().transpose() << "\n";
    std::cout << "Rotor p:         " << p.transpose() << "\n";
    std::cout << "Kf:              " << kf << "\n";
    std::cout << "Km:              " << km << std::endl;
    std::cout << "bTm t:           " << bTm.translation().transpose() << "\n";
    std::cout << "bTm r:           " << bTm.rotation().rpy().transpose() << "\n";
    std::cout << "drag k:          " << dk.transpose() << "\n";
    std::cout << "HoG k and HVT:             " << ak.transpose() << "\n";
    std::cout << "Viscous k:             " << bk.transpose() << "\n";

    return 0;
}
