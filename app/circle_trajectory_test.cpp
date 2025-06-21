#include "color.h"
#include "trajectory_generator/Trajectory_generator.h"
#include "quadrotor_simulator/Dynamics_control_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;
using namespace QuadrotorSim_SO3;
using namespace std;
using namespace Trajectory;
using namespace UAVFactor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(void)
{
    double dt = 0.001, radius = 1.0, linear_vel = 3.0, acc = 0.01;
    // circle_generator circle_generator(radius, linear_vel, dt);
    cir_conacc_generator circle_generator(radius, linear_vel, acc, dt);

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.05), Vector3::Constant(0.05), Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());

    DynamicsFactorTm dynamics_factor(X(0), V(0), S(0), U(0), X(1), V(1), S(1), 0.1f, dynamics_noise);

    Quadrotor quad;
    State state_0;
    State state_1, predicted_state;
    
    dt = 0.01;
    double t0 = 1.0; 
    // state_0.p = circle_generator.pos(t0);
    // state_0.v = circle_generator.vel(t0);
    // state_0.rot = Rot3::Expmap(circle_generator.theta(t0));
    // state_0.omega = circle_generator.omega(t0);
    // state_0.thrust_torque = circle_generator.inputfm(t0);

    quad.setState(state_0);

    gtsam::Vector4 last_input;

    for (int i = 0; i < 30000; i++)
    {
        gtsam::Vector4 input = circle_generator.inputfm(t0 + dt * (i + 1)); // circle_generator.inputfm(t0 + dt * (i + 1));

        gtsam::Vector4 input_ = circle_generator.input(t0 + dt * (i + 1));

        int b = 18700, a = 18500;
        gtsam::Vector4 rand_actuator;
        rand_actuator << (rand() % (b-a))+ a, (rand() % (b-a))+ a, (rand() % (b-a))+ a, (rand() % (b-a))+ a;

        input = quad.InvCumputeRotorsVel(rand_actuator);

        if(i == 0)
        {
            last_input = input;
        }

        std::cout << "Thrust moments: " << input.transpose() << std::endl;
        quad.stepODE(dt, input);

        last_input = input;

        if (i == 9)
        {
            state_1 = quad.getState();
        }

        std::cout << "***********************************************************************" << std::endl;
        std::cout << " predicted_state_x:"
                  << quad.getState().p.transpose() << " \n state_r:"
                  << Rot3::Logmap(quad.getState().rot).transpose() << " \n state_v:"
                  << quad.getState().v.transpose() << " \n state_omega:"
                  << quad.getState().body_rate.transpose() << std::endl;

        std::cout << " state_pos1:   [ " << circle_generator.pos(t0 + dt * i)[0] << " ," << circle_generator.pos(t0 + dt * i)[1] << " ," << circle_generator.pos(t0 + dt * i)[2] << " ]" << std::endl;
        std::cout << " state_vel1:   [ " << circle_generator.vel(t0 + dt * i)[0] << " ," << circle_generator.vel(t0 + dt * i)[1] << " ," << circle_generator.vel(t0 + dt * i)[2] << " ]" << std::endl;
        std::cout << " state_theta1: [ " << circle_generator.theta(t0 + dt * i)[0] << " ," << circle_generator.theta(t0 + dt * i)[1] << " ," << circle_generator.theta(t0 + dt * i)[2] << " ]" << std::endl;
        std::cout << " state_omega1: [ " << circle_generator.omega(t0 + dt * i)[0] << " ," << circle_generator.omega(t0 + dt * i)[1] << " ," << circle_generator.omega(t0 + dt * i)[2] << " ]" << std::endl;

        quad.renderHistoryTrj();
        std::cout << "***********************************************************************" << std::endl;
    }

    gtsam::Vector4 input = circle_generator.inputfm(t0);

    Pose3 pose_i(state_0.rot, state_0.p), pose_j(state_1.rot, state_1.p);
    Vector3 vel_i(state_0.v), vel_j(state_1.v), omega_i(state_0.body_rate), omega_j(state_1.body_rate);

    std::cout << "pose_i: \n"
              << pose_i << std::endl;
    std::cout << "pose_j: \n"
              << pose_j << std::endl;

    Matrix H_e_posei, H_e_posej;
    Matrix H_e_vi, H_e_oi, H_e_vj, H_e_oj;
    Matrix H_e_ui;

    Vector12 err = dynamics_factor.evaluateError(pose_i, vel_i, omega_i, input, pose_j, vel_j, omega_j, H_e_posei, H_e_vi, H_e_oi, H_e_ui, H_e_posej, H_e_vj, H_e_oj);
    std::cout << "err: " << err.transpose() << std::endl;

    while (true)
    {
        quad.renderHistoryTrj();
    }

    return 0;
}
