#include "calibration/Dynamics_calib.h"
#include "color.h"
#include "quadrotor_simulator/Dynamics_control_factor.h"
#include "quadrotor_simulator/Dynamics_params.h"
#include "trajectory_generator/Trajectory_generator.h"
#include <yaml-cpp/yaml.h>
#include "quadrotor_simulator/Quadrotor_SO3.h"

using namespace Calib;
using namespace gtsam;
using namespace std;
using namespace UAVFactor;
using namespace QuadrotorSim_SO3;
using namespace Trajectory;

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
    // quad_params.mass       = 0.915f;
    // quad_params.Ixx        = 4.9* 1e-2;// + 2* 1e-2;
    // quad_params.Iyy        = 4.9* 1e-2;// + 0.03;
    // quad_params.Izz        = 6.0* 1e-2;// + 0.01;
    quad_params.k_f        = quad_params.k_f* 1.5; 
    quad_params.k_m        = quad_params.k_m* 2.1;
    // quad_params.esc_factor = 1;
    quad_params.arm_length = 0.20;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    gtsam::Vector3 trust_err(0.001, 0.001, 0.001);
    gtsam::Vector3 moments_err(1e-6, 1e-6, 1e-6);

    gtsam::Vector3 trust_err2(0.1, 0.1, 0.1);
    gtsam::Vector3 moments_err2(1e-5, 1e-2, 1e-5);

    double dtt = 0.001, radius = 1.0, linear_vel = 3.0, acc = 0.01;
    circle_generator circle_generator(radius, linear_vel, dtt);
    // cir_conacc_generator circle_generator(radius, linear_vel, acc, dt);

    Quadrotor quad;
    Quadrotor::State state_0;
    Quadrotor::State state_1;
    
    dt = 0.01;
    double t0 = 1.0; 
    state_0.x = circle_generator.pos(t0);
    state_0.v = circle_generator.vel(t0);
    state_0.rot = Rot3::Expmap(circle_generator.theta(t0));
    state_0.omega = circle_generator.omega(t0);
    state_0.force_moment = circle_generator.inputfm(t0);

    quad.setState(state_0);

    auto dynamics_noise         = noiseModel::Diagonal::Sigmas((Vector(12) << trust_err* 0.5f* dt *dt, Vector3::Constant(0.0001), 
                                                                              trust_err* dt, moments_err* dt).finished());
    // auto thrust_torque_noise    = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.01), Vector3::Constant(1e-6)).finished());
    auto thrust_torque_noise    = noiseModel::Diagonal::Sigmas((Vector(6) << trust_err2, moments_err2).finished());

    auto vicon_noise            = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());

    auto vel_noise              = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    auto omega_noise            = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    auto im_noise               = noiseModel::Diagonal::Sigmas(Vector3(0.00001, 0.00001, 0.00001));
    auto tm_noise               = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.0001), Vector3::Constant(0.000001)).finished());

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
    uint32_t DATASET_LENS = 2000;

    std::vector<State>   Interp_states;
    std::vector<State>   Uav_states;
    std::vector<Uav_pwm> Uav_pwms;
    std::vector<gtsam::Vector6> trust_momentss;

    for (int i = 0; i < 30000; i++)
    {
        gtsam::Vector4 input_ = circle_generator.input(t0 + dt * (i + 1));
        std::cout << "input" << input_ << "\n";

        gtsam::Vector4 trust_moment4 = circle_generator.inputfm(t0 + dt * (i + 1));
        gtsam::Vector6 trust_moment6;
        trust_moment6 << 0, 0, trust_moment4(0), trust_moment4(1), trust_moment4(2), trust_moment4(3);

        trust_momentss.push_back(trust_moment6);

        state_1 = quad.getState();
        Uav_pwm _uav_pwm;
        _uav_pwm.timestamp       = t0 + dt * (i + 1);
        _uav_pwm.actuator_output = input_;
        Uav_pwms.push_back(_uav_pwm);

        State _state;
        _state.timestamp   = t0 + dt * (i + 1);
        _state.pose        = gtsam::Pose3(state_1.rot, state_1.x);
        _state.vel         = state_1.v;
        _state.omega       = state_1.omega;
        _state.actuator_output = input_;

        gtsam::Rot3 rx_pi  = gtsam::Rot3::Rx(M_PI);
        gtsam::Rot3 rz_pi4 = gtsam::Rot3::Rz(M_PI/4.0);

        // _state.pose       =  _state.pose* gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));// * gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));
        Interp_states.push_back(_state);
    }
    
    State _interp_state;

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
        
        // std::cout << "before add dyn factor \n";
        // pose, velocity, angular speed, thrust_moments, pose, velocity, angular speed, inertial of moments, rot of g
        DynamcisCaliFactor_TM dynamicsCalibFactor(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), dt, quad_params.mass, dynamics_noise);

        // DynamcisCaliFactorthrustMoments8 dynamicsCalibFactor(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), dt, quad_params.mass, dynamics_noise);

        // std::cout << "after estab dyn factor \n";

        dyn_factor_graph.add(dynamicsCalibFactor);

        // std::cout << "before add act factor \n";

        // thrust and moments, position of rotor, kf, km, esc_factor (0 << esc_factor <= 1)
        // if esc_factor is approaching 1, the observability of esc_factor will decrease. 
        AllocationCalibFactor3 allocation_factor(T(idx), P(0), K(0), M(0), Interp_states.at(idx).actuator_output, thrust_torque_noise);
        dyn_factor_graph.add(allocation_factor);

        gtsam::Vector6 trust_m;
        initial_value_dyn.insert(T(idx), trust_momentss[idx]);
        dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector6>(T(idx), trust_momentss[idx], tm_noise));
    }

    initial_value_dyn.insert(J(0), gtsam::Vector3(quad_params.Ixx* 1.3, quad_params.Iyy, quad_params.Izz* 1.4) );
    initial_value_dyn.insert(R(0), gtsam::Rot3::Rx(-1.0/180.0*M_PI)* gtsam::Rot3::Rx(2.0/180.0*M_PI));
    initial_value_dyn.insert(K(0), quad_params.k_f);
    initial_value_dyn.insert(M(0), quad_params.k_m);
    
    // gtsam::Vector3 rotor_p(-0., 0.115f, 0);
    gtsam::Vector3 rotor_p(18, 17, 0);
    

    initial_value_dyn.insert(P(0), rotor_p);
    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz), im_noise));
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
        
        // std::cout << idx << " - trust torque: " << t_t.transpose() << "\n";
        gtsam::Vector6 tt_err = allo_for_err.evaluateError(t_t, p, kf, mf);
        // JEC_log << idx << " " << t_t(0) << " " << t_t(1) << " " << t_t(2) << " " << t_t(3) << " " << t_t(4) << " " << t_t(5) << " " << tt_err(0) << " " << tt_err(1) << " " << tt_err(2) << " " << tt_err(3) << " " << tt_err(4) << " " << tt_err(5) << " \n";

        DynamcisCaliFactor_TM dyn_err(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), dt, quad_params.mass, dynamics_noise);
        gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, Interp_states.at(idx).vel, Interp_states.at(idx).omega, t_t, Interp_states.at(idx+1).pose, Interp_states.at(idx+1).vel, Interp_states.at(idx+1).omega, IM, rot);
        gtsam::Vector6 real_tm = trust_momentss.at(idx);

        JEC_log << idx << " " << t_t(0) << " " << t_t(1) << " " << t_t(2) << " " << t_t(3) << " " << t_t(4) << " " << t_t(5) << " " << tt_err(0) << " " << tt_err(1) << " " << tt_err(2) << " " << tt_err(3) << " " << tt_err(4) << " " << tt_err(5) << " " << 
        dyn_e(0) << " " << dyn_e(1) << " " << dyn_e(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) << " " << real_tm(0) << " " << real_tm(1) << " " << real_tm(2) << " " << real_tm(3) << " " << real_tm(4) << " " << real_tm(5) << "\n";
    }

    std::cout << "Inertial moment: " << IM.transpose() << "\n";
    std::cout << "Rot:             " << rot.rpy().transpose() << "\n";
    std::cout << "Rotor p:         " << p.transpose() << "\n";
    std::cout << "KF:              " << kf << "\n";
    std::cout << "MF:              " << mf << std::endl;

    return 0;
}
