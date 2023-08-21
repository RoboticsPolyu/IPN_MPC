#include "calibration/Dynamics_calib.h"
#include "color.h"
#include "quadrotor_simulator/Dynamics_control_factor.h"
#include "quadrotor_simulator/Dynamics_params.h"

#include <yaml-cpp/yaml.h>

using namespace Calib;
using namespace gtsam;
using namespace std;
using namespace UAVFactor;

using symbol_shorthand::E; // esc_factor
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

typedef struct State
{
    int            id;
    double         timestamp;
    gtsam::Pose3   pose;
    gtsam::Vector3 vel;
    gtsam::Vector3 omega;
    gtsam::Vector4 actuator_output;

} State;

typedef struct Uav_pwm
{
    int            id;
    double         timestamp;
    gtsam::Vector4 actuator_output;

} Uav_pwm;

// Pose slerp interpolation
Pose3 interpolateRt(const::Pose3& T_l, const Pose3& T, double t) 
{
    return Pose3(gtsam::interpolate<Rot3>(T_l.rotation(), T.rotation(), t), 
        gtsam::interpolate<Point3>(T_l.translation(), T.translation(), t));
}

int main(void)
{    
    // Configuration file 
    YAML::Node FGO_config = YAML::LoadFile("../config/dynamics_calib.yaml");  

    // Model predictive control duration
    DynamicsParams quad_params;
    double      dt         = 0.01f;
    quad_params.mass       = 0.915f;
    quad_params.Ixx        = 4.9* 1e-2;// + 2* 1e-2;
    quad_params.Iyy        = 4.9* 1e-2;// + 0.03;
    quad_params.Izz        = 6.0* 1e-2;// + 0.01;
    quad_params.k_f        = 2.27* 10e-8 + 1.5* 10e-8; 
    quad_params.k_m        = 0;
    quad_params.arm_length = 0.13;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    double assumed_thrust_sigma = 0.1; // N
    double assumed_dragf_sigma  = 0.01;
    gtsam::Vector3 thrust_err(assumed_dragf_sigma, assumed_dragf_sigma, assumed_thrust_sigma);
    gtsam::Vector3 moments_err(0.1f* assumed_thrust_sigma, 0.1f* assumed_thrust_sigma, 0.01f* assumed_thrust_sigma);

    auto dynamics_noise      = noiseModel::Diagonal::Sigmas((Vector(12) << thrust_err * 0.5f * dt * dt, Vector3::Constant(0.0001), 
                                                                        thrust_err * dt, moments_err * dt).finished());

    auto vicon_noise         = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());

    auto im_noise            = noiseModel::Diagonal::Sigmas(Vector3(0.00001, 0.00001, 0.00001));
    auto rp_noise            = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto gr_noise            = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto km_noise            = noiseModel::Diagonal::Sigmas(Vector1(0.00001));

    std::ofstream JEC_log;
    std::string file_name = "../data/calib_";
    file_name.append("debug");
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    NonlinearFactorGraph dyn_factor_graph;
    Values initial_value_dyn;
    dyn_factor_graph.empty();
    
    // Load dataset
    uint32_t DATASET_LENS = 2000;

    std::vector<State>   Interp_states;
    std::vector<State>   Uav_states;
    std::vector<Uav_pwm> Uav_pwms;

    std::ifstream state_file;
    state_file.open("/Users/ypwen/IPN/IPN_MPC/data/blackbird/pacasso/yawForward/maxSpeed4p0/state_black.txt");
    std::ifstream actuator_black_file;
    actuator_black_file.open("/Users/ypwen/IPN/IPN_MPC/data/blackbird/pacasso/yawForward/maxSpeed4p0/interp_actuator_black.txt");
    std::ifstream rpm_black_file;
    rpm_black_file.open("/Users/ypwen/IPN/IPN_MPC/data/blackbird/pacasso/yawForward/maxSpeed4p0/rpm_black.txt");

    double gt_t, gt_qw, gt_qx, gt_qy, gt_qz, gt_x, gt_y, gt_z, pwm_t, pwm1, pwm2, pwm3, pwm4;

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));

    while (state_file >> gt_t >> gt_x >> gt_y >> gt_z >> gt_qw >> gt_qx >> gt_qy >> gt_qz)
    {
        State _state;
        _state.timestamp   = gt_t;
        _state.pose        = gtsam::Pose3(gtsam::Quaternion(gt_qw, gt_qx, gt_qy, gt_qz), gtsam::Vector3(gt_x, gt_y, gt_z));
        gtsam::Rot3 rx_pi  = gtsam::Rot3::Rx(M_PI);
        gtsam::Rot3 rz_pi4 = gtsam::Rot3::Rz(M_PI/4.0);

        _state.pose       = gtsam::Pose3(rx_pi, gtsam::Vector3(0,0,0))* _state.pose * gtsam::Pose3(rx_pi, gtsam::Vector3(0,0,0));// * gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));
        Uav_states.push_back(_state);
    }

    while (rpm_black_file >> pwm_t >> pwm1 >> pwm2 >> pwm3 >> pwm4)
    {
        Uav_pwm _uav_pwm;
        _uav_pwm.timestamp      = pwm_t;
        _uav_pwm.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);
        Uav_pwms.push_back(_uav_pwm);
    }

    // while (actuator_black_file >> pwm_t >> pwm1 >> pwm2 >> pwm3 >> pwm4)
    // {
    //     Uav_pwm _uav_pwm;
    //     _uav_pwm.timestamp      = pwm_t;
    //     _uav_pwm.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);
    //     Uav_pwms.push_back(_uav_pwm);
    // }

    State _interp_state;

    for(int i = 1200; i < 20000; i++)
    {
        for(int j = 0; j < Uav_states.size() -1; j++)
        {
            if(Uav_pwms[i].timestamp >= Uav_states[j].timestamp && Uav_pwms[i].timestamp < Uav_states[j+1].timestamp)
            {
                double t = (Uav_pwms[i].timestamp - Uav_states[j].timestamp) / (Uav_states[j+1].timestamp - Uav_states[j].timestamp);

                gtsam::Pose3 interp_pose = interpolateRt(Uav_states[j].pose, Uav_states[j+1].pose, t);
                
                _interp_state.actuator_output = Uav_pwms[i].actuator_output;
                _interp_state.timestamp       = Uav_pwms[i].timestamp;
                _interp_state.pose            = interp_pose;

                gtsam::Vector3 vel_l   = (Uav_states.at(j+1).pose.translation() - Uav_states.at(j-1).pose.translation())/dt/2.0;
                gtsam::Vector3 vel_r   = (Uav_states.at(j+2).pose.translation() - Uav_states.at(j).pose.translation())/dt/2.0;

                gtsam::Vector3 omega_l = gtsam::Rot3::Logmap(Uav_states.at(j-1).pose.rotation().between(Uav_states.at(j+1).pose.rotation()))/2.0 /dt;
                gtsam::Vector3 omega_r = gtsam::Rot3::Logmap(Uav_states.at(j).pose.rotation().between(Uav_states.at(j+2).pose.rotation()))/2.0 /dt;

                _interp_state.vel      = (1-t)* vel_l + t* vel_r;
                _interp_state.omega    = (1-t)* omega_l + t* omega_r;

                Interp_states.push_back(_interp_state);
            }
        }
    }


    for(uint32_t idx = 100; idx < DATASET_LENS; idx++)
    {   
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;

        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), Interp_states.at(idx).pose, vicon_noise));

        // std::cout << idx << " " << std::setprecision(19) << Interp_states.at(idx).timestamp << " " << std::setprecision(5) << vel.transpose() << " " << omega.transpose() << "\n";

        initial_value_dyn.insert(X(idx), Interp_states.at(idx).pose);
        initial_value_dyn.insert(V(idx), Interp_states.at(idx).vel);
        initial_value_dyn.insert(S(idx), Interp_states.at(idx).omega);

        if( idx == DATASET_LENS-1)
        {
            dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx+1), Interp_states.at(idx+1).pose,  vicon_noise));

            initial_value_dyn.insert(X(idx+1), Interp_states.at(idx+1).pose);
            initial_value_dyn.insert(V(idx+1), Interp_states.at(idx+1).vel);
            initial_value_dyn.insert(S(idx+1), Interp_states.at(idx+1).omega);
        }
        
        // pose, velocity, angular speed, pose, velocity, angular speed, inertial of moments, rot of g, position of rotot, kf, km        
        DynamcisCaliFactor_RS dynamicsCalibFactor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dynamics_noise);
        dyn_factor_graph.add(dynamicsCalibFactor);
    }

    initial_value_dyn.insert(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz) );
    initial_value_dyn.insert(R(0), gtsam::Rot3::Ry(-3.0/180.0*M_PI)* gtsam::Rot3::Rx(5.0/180.0*M_PI));
    initial_value_dyn.insert(K(0), quad_params.k_f);
    initial_value_dyn.insert(M(0), quad_params.k_m);
    
    gtsam::Vector3 rotor_p(quad_params.arm_length/ std::sqrt(2), quad_params.arm_length/ std::sqrt(2), 0);
    
    initial_value_dyn.insert(P(0), rotor_p);
    dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz), im_noise));
    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Rot3>(R(0), gtsam::Rot3::identity(), gr_noise));

    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(P(0), rotor_p, rp_noise)); 
    // dyn_factor_graph.add(gtsam::PriorFactor<double>(M(0), 0.0025f, km_noise));

    std::cout << "###################### init contoller optimizer ######################" << std::endl;
    LevenbergMarquardtOptimizer optimizer(dyn_factor_graph, initial_value_dyn, parameters);
    std::cout << "###################### begin optimize ######################" << std::endl;
    Values result = optimizer.optimize();
    

    gtsam::Vector3  IM = result.at<gtsam::Vector3>(J(0));
    gtsam::Rot3    rot = result.at<gtsam::Rot3>(R(0));
    gtsam::Vector3   p = result.at<gtsam::Vector3>(P(0));
    double          kf = result.at<double>(K(0));
    double          mf = result.at<double>(M(0)); 
    
    
    for(uint32_t idx = 100; idx < DATASET_LENS; idx++)
    {
        DynamcisCaliFactor_RS dyn_err(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dynamics_noise);
        gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, Interp_states.at(idx).vel, Interp_states.at(idx).omega, Interp_states.at(idx+1).pose, Interp_states.at(idx+1).vel, Interp_states.at(idx+1).omega, IM, rot, p, kf, mf);

        // JEC_log << idx << " " << t_t(0) << " " << t_t(1) << " " << t_t(2) << " " << t_t(3) << " " << t_t(4) << " " << t_t(5) << " " << tt_err(0) << " " << tt_err(1) << " " << tt_err(2) << " " << tt_err(3) << " " << tt_err(4) << " " << tt_err(5) << " " << 
        // dyn_e(0) << " " << dyn_e(1) << " " << dyn_e(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) << " \n";
        gtsam::Vector3 rpy = Interp_states.at(idx).pose.rotation().rpy();

    //    JEC_log << idx << " " << dyn_e(0) << " " << dyn_e(1) << " " << dyn_e(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) << " " << Interp_states.at(idx).actuator_output[0] << " " << Interp_states.at(idx).actuator_output[1] << " " << Interp_states.at(idx).actuator_output[2] << " " << Interp_states.at(idx).actuator_output[3] << " " << rpy(0) << " " << rpy(1) << " " << rpy(2) << " \n";

        JEC_log << dyn_e(0) << " " << dyn_e(1) << " " << dyn_e(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) << "\n";


    }

    std::cout << "Inertial of moment: " << IM.transpose() << "\n";
    std::cout << "Rot:             " << rot.rpy().transpose() << "\n";
    std::cout << "Rotor p:         " << p.transpose() << "\n";
    std::cout << "Kf:              " << kf << "\n";
    std::cout << "Km:              " << mf << std::endl;

    return 0;
}
