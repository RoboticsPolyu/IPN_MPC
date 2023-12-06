#include "calibration/Calibration_factor.h"
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
using symbol_shorthand::D; // body_T_mass

typedef struct State
{
    int            id;
    double         timestamp;
    gtsam::Pose3   pose;
    gtsam::Vector3 vel;
    gtsam::Vector3 omega;
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
    return Pose3(gtsam::interpolate<Rot3>(T_l.rotation(), T.rotation(), t), 
        gtsam::interpolate<Point3>(T_l.translation(), T.translation(), t));
}

int main(void)
{    
    // Configuration file 
    YAML::Node DYNC_config = YAML::LoadFile("../config/dynamics_calib.yaml");  
    double rotor_p_x       = DYNC_config["ROTOR_P_X"].as<double>();
    double rotor_p_y       = DYNC_config["ROTOR_P_Y"].as<double>();
    DynamicsParams quad_params;
    double dt              = 0.01f;
    quad_params.Ixx        = 4.9* 1e-3;
    quad_params.Iyy        = 4.9* 1e-3;
    quad_params.Izz        = 10.0* 1e-3;
    quad_params.k_f        = quad_params.k_f* 1.5; 
    quad_params.k_m        = quad_params.k_m* 4;
    quad_params.arm_length = 0.20;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    double p_thrust_sigma         = 0.1f;
    double unmodeled_thrust_sigma = 0.01f; 

    gtsam::Vector3 thrust_sigma(unmodeled_thrust_sigma, unmodeled_thrust_sigma, 2* p_thrust_sigma);
    // sigma = (rotor_p_x* 2* P_thrust_single_rotor, rotor_p_x* 2* P_thrust_single_rotor, k_m * 2 * P_thrust_single_rotor) 
    gtsam::Vector3 moments_sigma(p_thrust_sigma * 0.2f, p_thrust_sigma * 0.2f, 0.026f * p_thrust_sigma);

    double dtt = 0.001, radius = 1.0, linear_vel = 3.0, acc = 0.01;
    // circle_generator circle_generator(radius, linear_vel, dtt);
    cir_conacc_generator circle_generator(radius, linear_vel, acc, dtt);
    
    double t0 = 1.0; 

    auto dyn_noise   = noiseModel::Diagonal::Sigmas((Vector(12) << thrust_sigma * 0.5f * dt * dt, Vector3::Constant(0.001), 
                                                                   thrust_sigma * dt, moments_sigma * dt).finished());
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());

    auto im_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.00001, 0.00001, 0.00001));
    auto rp_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto gr_noise    = noiseModel::Diagonal::Sigmas(Vector3(0.0001, 0.0001, 0.0001));
    auto km_noise    = noiseModel::Diagonal::Sigmas(Vector1(0.00001));
    auto bm_nosie    = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.00001), Vector3::Constant(0.00001)).finished());

    std::ofstream calib_log;
    std::string file_name = "../data/calib_";
    file_name.append("debug");
    file_name.append("_log.txt");
    calib_log.open(file_name);

    NonlinearFactorGraph dyn_factor_graph;
    Values initial_value_dyn;
    dyn_factor_graph.empty();
    
    // Load dataset
    uint32_t DATASET_LENS = 2000;

    std::vector<State>   Interp_states;
    std::vector<State>   Uav_states;
    std::vector<Actuator_control> Uav_pwms;

    Quadrotor quad;
    Quadrotor::State state_0;
    Quadrotor::State state_1;

    state_0.p     = gtsam::Vector3(0,0,0);
    state_0.rot   = gtsam::Rot3::identity();
    state_0.v     = gtsam::Vector3(0,0,0);
    state_0.omega = gtsam::Vector3(0,0,0);
    
    quad.setState(state_0);

    for (int i = 0; i < DATASET_LENS+1000; i++)
    {
        float timestamp_cur = t0 + dt * (i + 1);

        // Quadrotor simulation
        int b = 16660, a = 16430;
        gtsam::Vector4 rand_actuator;
        rand_actuator << (rand() % (b-a))+ a, (rand() % (b-a))+ a, (rand() % (b-a))+ a, (rand() % (b-a))+ a;
        // std::cout << "Rotor speed: " << rand_actuator.transpose() << std::endl;
        gtsam::Vector4 thrust_moments = quad.InvCumputeRotorsVel(rand_actuator);
        // std::cout << "Thrust moments: " << thrust_moments.transpose() << std::endl;
        quad.stepODE(dt, thrust_moments);
        state_1 = quad.getState();
 
        // gtsam::Vector4 input_ = circle_generator.input(timestamp_cur);
        // gtsam::Vector4 thrust_moment4 = circle_generator.inputfm(timestamp_cur);

        Actuator_control _uav_pwm;
        _uav_pwm.timestamp       = timestamp_cur;
        _uav_pwm.actuator_output = rand_actuator; // input_
        Uav_pwms.push_back(_uav_pwm);

        State _state;
        _state.timestamp   = timestamp_cur;
        _state.pose        = gtsam::Pose3(state_1.rot, state_1.p);
        // gtsam::Pose3(Rot3::Expmap(circle_generator.theta(timestamp_cur)),circle_generator.pos(timestamp_cur));
        _state.vel         = state_1.v; // circle_generator.vel(timestamp_cur);
        _state.body_rate   = state_1.body_rate; // circle_generator.omega(timestamp_cur);

        _state.actuator_output = rand_actuator;// input_;

        // _state.timestamp   = timestamp_cur;
        // _state.pose        = gtsam::Pose3(Rot3::Expmap(circle_generator.theta(timestamp_cur)),circle_generator.pos(timestamp_cur));
        // _state.vel         = circle_generator.vel(timestamp_cur);
        // _state.body_rate       = circle_generator.omega(timestamp_cur);

        // _state.actuator_output = input_;

        gtsam::Rot3 rx_pi  = gtsam::Rot3::Rx(M_PI);
        gtsam::Rot3 rz_pi4 = gtsam::Rot3::Rz(M_PI/4.0);

        // _state.pose     =  _state.pose* gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));// * gtsam::Pose3(rz_pi4, gtsam::Vector3(0,0,0));
        Interp_states.push_back(_state);
        quad.renderHistoryTrj();
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
        initial_value_dyn.insert(S(idx), Interp_states.at(idx).body_rate);

        if( idx == DATASET_LENS - 1 )
        {
            dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx+1), Interp_states.at(idx+1).pose,  vicon_noise));
            initial_value_dyn.insert(X(idx+1), Interp_states.at(idx+1).pose);
            initial_value_dyn.insert(V(idx+1), Interp_states.at(idx+1).vel);
            initial_value_dyn.insert(S(idx+1), Interp_states.at(idx+1).body_rate);
        }
        
        // pose, velocity, angular speed, thrust_moments, pose, velocity, angular speed, inertial of moments, rot of g
        DynamcisCaliFactor_RS dynamicsCalibFactor(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), D(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);
        dyn_factor_graph.add(dynamicsCalibFactor);
    }

    initial_value_dyn.insert(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz) );
    initial_value_dyn.insert(R(0), gtsam::Rot3::Rz(0.0/180.0*M_PI)* gtsam::Rot3::Ry(-3.0/180.0*M_PI)* gtsam::Rot3::Rx(5.0/180.0*M_PI));
    initial_value_dyn.insert(K(0), quad_params.k_f);
    initial_value_dyn.insert(M(0), quad_params.k_m);
    gtsam::Rot3 delta_ = gtsam::Rot3::Rz(4.0/180.0*M_PI)* gtsam::Rot3::Ry(-3.0/180.0*M_PI)* gtsam::Rot3::Rx(5.0/180.0*M_PI);
    initial_value_dyn.insert(D(0), gtsam::Pose3::identity() * gtsam::Pose3(delta_, gtsam::Vector3(0.10f, 0.10f, 0.10f)));

    gtsam::Vector3 rotor_p(rotor_p_x, rotor_p_y, 0);
    // gtsam::Vector3 fake_rotor_p(0.15f, 0.20f, 0);
    initial_value_dyn.insert(P(0), rotor_p);
    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(P(0), rotor_p, rp_noise)); 
    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(D(0), gtsam::Pose3::identity(), bm_nosie));
    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Vector3>(J(0), gtsam::Vector3(quad_params.Ixx, quad_params.Iyy, quad_params.Izz), im_noise));
    // dyn_factor_graph.add(gtsam::PriorFactor<gtsam::Rot3>(R(0), gtsam::Rot3::identity(), gr_noise));
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
    gtsam::Pose3   bTm = result.at<gtsam::Pose3>(D(0)); 

    
    for(uint32_t idx = 100; idx < DATASET_LENS; idx++)
    {
        // AllocationCalibFactor3 allo_for_err(T(idx), P(0), K(0), M(0), Interp_states.at(idx).actuator_output, thrust_torque_noise);
        // gtsam::Vector6 t_t = result.at<gtsam::Vector6>(T(idx));
        DynamcisCaliFactor_RS dyn_err(X(idx), V(idx), S(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), P(0), K(0), M(0), D(0), Interp_states.at(idx).actuator_output, dt, quad_params.mass, dyn_noise);
        // DynamcisCaliFactor_TM dyn_err(X(idx), V(idx), S(idx), T(idx), X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), dt, quad_params.mass, dyn_noise);
        gtsam::Vector12 dyn_e = dyn_err.evaluateError(Interp_states.at(idx).pose, Interp_states.at(idx).vel, Interp_states.at(idx).body_rate, Interp_states.at(idx+1).pose, Interp_states.at(idx+1).vel, Interp_states.at(idx+1).body_rate, IM, rot, p, kf, mf, bTm);

        gtsam::Pose3          pose = result.at<gtsam::Pose3>(X(idx));
        gtsam::Vector3 dyn_pos_err = pose.rotation() * gtsam::Vector3(dyn_e(0), dyn_e(1), dyn_e(2)) / quad_params.mass;

        calib_log << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) << "\n";
    }

    std::cout << "Inertial moment: " << IM.transpose() << "\n";
    std::cout << "Rot:          " << rot.rpy().transpose() << "\n";
    std::cout << "Rotor p:         " << p.transpose() << "\n";
    std::cout << "Kf:              " << kf << "\n";
    std::cout << "Km:              " << mf << std::endl;
    std::cout << "bTm t:           " << bTm.translation().transpose() << "\n";
    std::cout << "bTm r:           " << bTm.rotation().rpy().transpose() << "\n";
    return 0;
}
