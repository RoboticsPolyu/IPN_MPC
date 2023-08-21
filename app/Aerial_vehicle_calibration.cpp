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
    quad_params.k_f        = 2.27* 10e-8 + 2* 10e-8; 
    quad_params.k_m        = 0;
    quad_params.esc_factor = 1;
    quad_params.arm_length = 0.13;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    auto dynamics_noise         = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.01*0.01*0.5*0.2), Vector3::Constant(0.00001), 
                                                                              Vector3::Constant(0.01* 0.2), Vector3::Constant(0.001)).finished());
    // auto thrust_torque_noise    = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.01), Vector3::Constant(1e-6)).finished());
    auto thrust_torque_noise    = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(1e-6)).finished());
    auto vicon_noise            = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());


    auto im_noise               = noiseModel::Diagonal::Sigmas(Vector3(0.00001, 0.00001, 0.00001));
    auto rp_noise               = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));

    auto gr_noise               = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));

    auto km_noise               = noiseModel::Diagonal::Sigmas(Vector1(0.00001));

    std::ofstream JEC_log;
    std::string file_name = "../data/calib_";
    file_name.append("debug");
    file_name.append("_log.txt");
    JEC_log.open(file_name);

    NonlinearFactorGraph dyn_factor_graph;
    Values initial_value_dyn;
    dyn_factor_graph.empty();
    
    // Load dataset
    uint32_t DATASET_LENS = 10000;

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

        // _state.pose       = gtsam::Pose3(rx_pi, gtsam::Vector3(0,0,0))* _state.pose* gtsam::Pose3(rx_pi, gtsam::Vector3(0,0,0));// * gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));
        Uav_states.push_back(_state);
    }

    while (rpm_black_file >> pwm_t >> pwm1 >> pwm2 >> pwm3 >> pwm4)
    {
        Uav_pwm _uav_pwm;
        _uav_pwm.timestamp      = pwm_t;
        _uav_pwm.actuator_output = gtsam::Vector4(pwm1, pwm2, pwm3, pwm4);
        Uav_pwms.push_back(_uav_pwm);
    }


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

    // std::cout << "after interp \n";

    // std::vector<gtsam::Vector4> interp_actuator_outputs;
    // std::vector<gtsam::Pose3>   interp_poses;

    for(uint32_t idx = 100; idx < DATASET_LENS; idx++)
    {   
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;

        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), Interp_states.at(idx).pose, vicon_noise));
        // vel = (Interp_states.at(idx+1).pose.translation() - Interp_states.at(idx-1).pose.translation()) / dt/2;
        // omega = gtsam::Rot3::Logmap(gtsam::Rot3(Interp_states.at(idx+1).pose.rotation().inverse()* Interp_states.at(idx-1).pose.rotation())) / dt/2;
        // omega = gtsam::Rot3::Logmap(Interp_states.at(idx+1).pose.rotation().between(Interp_states.at(idx-1).pose.rotation())) / 2 / dt;


        // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), Interp_states.at(idx).vel,   vel_noise));
        // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), Interp_states.at(idx).omega, omega_noise));


        // std::cout << idx << " " << std::setprecision(19) << Interp_states.at(idx).timestamp << " " << std::setprecision(5) << vel.transpose() << " " << omega.transpose() << "\n";

        initial_value_dyn.insert(X(idx), Interp_states.at(idx).pose);
        initial_value_dyn.insert(V(idx), Interp_states.at(idx).vel);
        initial_value_dyn.insert(S(idx), Interp_states.at(idx).omega);

        if( idx == DATASET_LENS-1)
        {
            // vel = (Interp_states.at(idx+2).pose.translation() - Interp_states.at(idx).pose.translation()) / dt/2;
            // omega = gtsam::Rot3::Logmap(gtsam::Rot3(Interp_states.at(idx+2).pose.rotation().inverse()* Interp_states.at(idx).pose.rotation())) / dt/2;

            dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx+1), Interp_states.at(idx+1).pose,  vicon_noise));
            // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx+1), Interp_states.at(idx+1).vel,   vel_noise));
            // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx+1), Interp_states.at(idx+1).omega, omega_noise));

            initial_value_dyn.insert(X(idx+1), Interp_states.at(idx+1).pose);
            initial_value_dyn.insert(V(idx+1), Interp_states.at(idx+1).vel);
            initial_value_dyn.insert(S(idx+1), Interp_states.at(idx+1).omega);
        }
        
        // std::cout << "before add dyn factor \n";
        // pose, velocity, angular speed, thrust_moments, pose, velocity, angular speed, inertial of moments, rot of g
        DynamcisCaliFactorthrustMoments dynamicsCalibFactor(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), dt, quad_params.mass, dynamics_noise);

        // DynamcisCaliFactorthrustMoments8 dynamicsCalibFactor(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), dt, quad_params.mass, dynamics_noise);

        // std::cout << "after estab dyn factor \n";

        dyn_factor_graph.add(dynamicsCalibFactor);

        // std::cout << "before add act factor \n";

        // thrust and moments, position of rotor, kf, km, esc_factor (0 << esc_factor <= 1)
        // if esc_factor is approaching 1, the observability of esc_factor will decrease. 
        AllocationCalibFactor3 allocation_factor(T(idx), P(0), K(0), M(0), Interp_states.at(idx).actuator_output, thrust_torque_noise);
        dyn_factor_graph.add(allocation_factor);

        gtsam::Vector6 thrust_m;
        initial_value_dyn.insert(T(idx), thrust_m);
    }

    initial_value_dyn.insert(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz) );
    initial_value_dyn.insert(R(0), gtsam::Rot3::Rx(-1.0/180.0*M_PI)* gtsam::Rot3::Rx(2.0/180.0*M_PI));
    initial_value_dyn.insert(K(0), quad_params.k_f);
    initial_value_dyn.insert(M(0), quad_params.k_m);
    
    // gtsam::Vector3 rotor_p(-0., 0.115f, 0);
    gtsam::Vector3 rotor_p(-quad_params.arm_length/ std::sqrt(2), -quad_params.arm_length/ std::sqrt(2), 0);
    
    initial_value_dyn.insert(P(0), rotor_p);
    dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz), im_noise));
    dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Rot3>(R(0), gtsam::Rot3::identity(), gr_noise));

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
        AllocationCalibFactor3 allo_for_err(T(idx), P(0), K(0), M(0), Interp_states.at(idx).actuator_output, thrust_torque_noise);
        gtsam::Vector6 t_t = result.at<gtsam::Vector6>(T(idx));
        
        // std::cout << idx << " - thrust torque: " << t_t.transpose() << "\n";
        gtsam::Vector6 tt_err = allo_for_err.evaluateError(t_t, p, kf, mf);
        // JEC_log << idx << " " << t_t(0) << " " << t_t(1) << " " << t_t(2) << " " << t_t(3) << " " << t_t(4) << " " << t_t(5) << " " << tt_err(0) << " " << tt_err(1) << " " << tt_err(2) << " " << tt_err(3) << " " << tt_err(4) << " " << tt_err(5) << " \n";

        DynamcisCaliFactorthrustMoments dyn_err(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), dt, quad_params.mass, dynamics_noise);
        gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, Interp_states.at(idx).vel, Interp_states.at(idx).omega, t_t, Interp_states.at(idx+1).pose, Interp_states.at(idx+1).vel, Interp_states.at(idx+1).omega, IM, rot);

        JEC_log << idx << " " << t_t(0) << " " << t_t(1) << " " << t_t(2) << " " << t_t(3) << " " << t_t(4) << " " << t_t(5) << " " << tt_err(0) << " " << tt_err(1) << " " << tt_err(2) << " " << tt_err(3) << " " << tt_err(4) << " " << tt_err(5) << " " << 
        dyn_e(0) << " " << dyn_e(1) << " " << dyn_e(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) << " \n";
    }

    std::cout << "Inertial moment: " << IM.transpose() << "\n";
    std::cout << "Rot:             " << rot.rpy().transpose() << "\n";
    std::cout << "Rotor p:         " << p.transpose() << "\n";
    std::cout << "KF:              " << kf << "\n";
    std::cout << "MF:              " << mf << std::endl;

    return 0;
}
