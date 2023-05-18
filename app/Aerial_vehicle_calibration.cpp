#include "calibration/Dynamics_calib.h"
#include "color.h"
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

int main(void)
{
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier green(Color::FG_GREEN);
    
    // Configuration file 
    YAML::Node FGO_config = YAML::LoadFile("../config/dynamics_calib.yaml");  

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    
    auto thrust_moments_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), 
                                                            Vector3::Constant(1e-6)).finished());
    
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.005), Vector3::Constant(0.005)).finished());
    auto vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    double dt = 0.01f; // Model predictive control duration
    DynamicsParams sim_quad_params;

    NonlinearFactorGraph graph_dynamics_calib;
    Values initial_value_dynamics_calib;
    graph_dynamics_calib.empty();
    
    double PWM_MIN = 0;
    double PWM_MAX = 25000;

    uint32_t SIM_STEPS = 1000;
    for(uint32_t idx = 0; idx < SIM_STEPS; idx++)
    {   
        gtsam::Pose3   pose_idx;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 actuator_outputs;

        graph_dynamics_calib.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), pose_idx, vicon_noise));
        graph_dynamics_calib.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), vel, vel_noise));
        graph_dynamics_calib.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), omega, omega_noise));


        // pose, velocity, angular speed, thrust_moments, pose, velocity, angular speed, inertial of moments, rot of g
        DynamcisCaliFactorthrustMoments dynamicsCalibFactor(X(idx), V(idx), S(idx), T(idx), 
            X(idx + 1), V(idx + 1), S(idx + 1), J(0), R(0), dt, sim_quad_params.mass, thrust_moments_noise);
        graph_dynamics_calib.add(dynamicsCalibFactor);

        
        // thrust and moments, position of rotor, kf, km, esc_factor (0 << esc_factor <= 1)
        // if esc_factor is approaching 1, the observability of esc_factor will decrease. 
        AllocationCalibFactor3 allocation_factor(T(idx), P(0), K(0), M(0), E(0), actuator_outputs, PWM_MIN, PWM_MAX, 
            thrust_moments_noise);
        graph_dynamics_calib.add(allocation_factor);

    }
    
    std::cout << "###################### init contoller optimizer ######################" << std::endl;
    LevenbergMarquardtOptimizer optimizer(graph_dynamics_calib, initial_value_dynamics_calib, parameters);
    std::cout << "###################### begin optimize ######################" << std::endl;
    Values result = optimizer.optimize();

    return 0;
}
