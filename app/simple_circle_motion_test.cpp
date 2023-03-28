#include "trajectory_generator/Trajectory_generator.h"
#include "quadrotor_simulator/Dynamics_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;
using namespace QuadrotorSimulator_SO3;
using namespace std;
using namespace Trajectory;
using namespace uav_factor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(void)
{
    double dt = 0.01, radius = 1.0, linear_vel = 1.0;
    circle_generator circle_generator(radius, linear_vel, dt);

    std::vector<Quadrotor::State> reference_of_states;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations = 500;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

    NonlinearFactorGraph graph;

    auto input_noise = noiseModel::Diagonal::Sigmas(Vector3(500, 500, 500));
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.05), Vector3::Constant(0.05), Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.05), Vector3::Constant(0.05), Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());

    uint64_t key = 0;
    int opt_lens_traj = 20;

    // for (int idx = 0; idx < reference_of_states.size() - opt_lens_traj; idx++)
    // {
    //     DynamicsFactor2 dynamics_factor2(X(idx), U(idx), X(idx + 1), 0.01f, dynamics_noise);
    //     graph.add(dynamics_factor2);

    //     gtsam::Vector12 _state;
    //     gtsam::Vector4 _input; // reference_of_states[idx]
    //     if (idx != 0)
    //     {
    //         graph.add(gtsam::PriorFactor<gtsam::Vector12>(X(idx), _state, dynamics_noise));
    //         graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), _input, input_noise));
    //     }

    //     if (idx == 19)
    //     {
    //     }
    // }
    // Dynamics Factor Test

    DynamicsFactor dynamics_factor(X(0), V(0), S(0), U(0), X(1), V(1), S(1), 0.1f, dynamics_noise);

    Quadrotor quad;
    Quadrotor::State state_0 = quad.getState();
    Quadrotor::State state_1;

    double t0 = 1.0;
    state_0.x = circle_generator.pos(t0);
    state_0.v = circle_generator.vel(t0);
    state_0.rot = Rot3::Expmap(circle_generator.theta(t0));
    state_0.omega = circle_generator.omega(t0);
    Vector4 input = circle_generator.input(t0);
    state_0.motor_rpm = Eigen::Array4d(input[0], input[1], input[2], input[3]);

    quad.setState(state_0);

    for (int i = 0; i < 10000; i++)
    {
        Vector4 input = circle_generator.input(t0 + 0.01 * i);
        quad.setInput(input[0], input[1], input[2], input[3]);
        quad.step(0.01);
        state_1 = quad.getState();
        std::cout << "***********************************************************************" << std::endl;
        std::cout << "state_predicted:  \n"
                  << quad.getState().x.transpose() << " \n,state_r: \n"
                  << Rot3::Logmap(quad.getState().rot).transpose() << " \n,state_v:\n "
                  << quad.getState().v.transpose() << " \n,state_omega: \n"
                  << quad.getState().omega.transpose() << std::endl;

        std::cout << " state_pos1: [ " << circle_generator.pos(t0 + 0.01 * i)[0] << " ," << circle_generator.pos(t0 + 0.01 * i)[1] << " ," << circle_generator.pos(t0 + 0.01 * i)[2] << " ]" << std::endl;
        std::cout << " state_vel1: [ " << circle_generator.vel(t0 + 0.01 * i)[0] << " ," << circle_generator.vel(t0 + 0.01 * i)[1] << " ," << circle_generator.vel(t0 + 0.01 * i)[2] << " ]" << std::endl;
        std::cout << " state_theta1: [ " << circle_generator.theta(t0 + 0.01 * i)[0] << " ," << circle_generator.theta(t0 + 0.01 * i)[1] << " ," << circle_generator.theta(t0 + 0.01 * i)[2] << " ]" << std::endl;
        std::cout << " state_omega1: [ " << circle_generator.omega(t0 + 0.01 * i)[0] << " ," << circle_generator.omega(t0 + 0.01 * i)[1] << " ," << circle_generator.omega(t0 + 0.01 * i)[2] << " ]" << std::endl;
        quad.render();
        std::cout << "***********************************************************************" << std::endl;
    }

    input = circle_generator.input(t0);

    Pose3 pose_i(state_0.rot, state_0.x), pose_j(state_1.rot, state_1.x);
    Vector3 vel_i(state_0.v), vel_j(state_1.v), omega_i(state_0.omega), omega_j(state_1.omega);

    std::cout << "pose_i: \n" << pose_i << std::endl;
    std::cout << "pose_j: \n" << pose_j << std::endl;

    Matrix H_e_posei, H_e_posej;
    Matrix H_e_vi, H_e_oi, H_e_vj, H_e_oj;
    Matrix H_e_ui;

    Vector12 err = dynamics_factor.evaluateError(pose_i, vel_i, omega_i, input, pose_j, vel_j, omega_j, H_e_posei, H_e_vi, H_e_oi, H_e_ui, H_e_posej, H_e_vj, H_e_oj);
    std::cout << "err: " << err.transpose() << std::endl;

    int flag;
    flag = getchar(); //Pause

    return 0;
}
