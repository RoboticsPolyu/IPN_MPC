#include "color.h"
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
    double dt = 0.0001, radius = 1.0, linear_vel = 1.0;
    circle_generator circle_generator(radius, linear_vel, dt);

    std::vector<Quadrotor::State> ref_states;
    Quadrotor::State m_state;
    for (int idx = 0; idx < 2000; idx++)
    {
        double t = 0 + dt * idx;

        m_state.x = circle_generator.pos(t);
        m_state.v = circle_generator.vel(t);
        m_state.rot = gtsam::Rot3::Expmap(circle_generator.theta(t));
        m_state.omega = circle_generator.omega(t);
        gtsam::Vector4 input = circle_generator.input(t);
        m_state.motor_rpm << input[0], input[1], input[2], input[3];
        ref_states.push_back(m_state);
    }

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations = 500;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(2, 1e-3, 1e-3, 1e-3));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.005), Vector3::Constant(0.005), Vector3::Constant(0.005), Vector3::Constant(0.005)).finished());
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.01)).finished());
    auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(.3, .3, .3));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(.3, .3, .3));

    int opt_lens_traj = 20;

    dt = 0.01;
    Quadrotor quad_;
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier green(Color::FG_GREEN);

    Quadrotor::State state_predicted;

    for(int traj_idx = 0; traj_idx <100000; traj_idx++)
    {
        double t0 = traj_idx* dt;

        if(traj_idx == 0)
        {
            state_predicted.x =  circle_generator.pos(t0);//  + gtsam::Vector3(0.05, 0, 0.10);
            state_predicted.rot = gtsam::Rot3::Expmap(circle_generator.theta(t0));
            state_predicted.v = circle_generator.vel(t0);// gtsam::Vector3(0.05, 0, 0.10);
            state_predicted.omega = circle_generator.omega(t0);
            state_predicted.force_moment = circle_generator.inputfm(t0);
        }
        
        NonlinearFactorGraph graph;
        Values initial_value;
        graph.empty();

        for (int idx = 0; idx < opt_lens_traj; idx++)
        {
            DynamicsFactorfm dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1), V(idx + 1), S(idx + 1), dt, dynamics_noise);
            graph.add(dynamics_factor);
            
            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + (idx + 1) * dt)), circle_generator.pos(t0 + (idx + 1) * dt));
            gtsam::Vector3 vel_idx = circle_generator.vel(t0 + (idx + 1) * dt);
            gtsam::Vector3 omega_idx = circle_generator.omega(t0 + (idx + 1) * dt);

            initial_value.insert(X(idx + 1), pose_idx);
            initial_value.insert(V(idx + 1), vel_idx);
            initial_value.insert(S(idx + 1), omega_idx);
            gtsam::Vector4 diff_input;
            diff_input.setZero();
            // diff_input << 0.5, 5e-6, 5e-6, 5e-6;
            
            gtsam::Vector4 init_input = circle_generator.inputfm(t0 + idx * dt);
            init_input = init_input + diff_input;

            // graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            initial_value.insert(U(idx), init_input);

            graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));

            if (idx == 0)
            {
                std::cout << red;
                std::cout << "Init statex: " << state_predicted.x.transpose() << std::endl;
                std::cout << "Init stater: " << Rot3::Logmap(state_predicted.rot).transpose() << std::endl;
                std::cout << "Init statev: " << state_predicted.v.transpose() << std::endl;
                std::cout << "Init stateomega: " << state_predicted.omega.transpose() << std::endl;
                std::cout << def;

                graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), gtsam::Pose3(state_predicted.rot, state_predicted.x), vicon_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), state_predicted.v, vel_noise));
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), state_predicted.omega, omega_noise));

                initial_value.insert(X(idx), gtsam::Pose3(state_predicted.rot, state_predicted.x));
                initial_value.insert(V(idx), state_predicted.v);
                initial_value.insert(S(idx), state_predicted.omega);
            }

        }

        std::cout << "###################### init optimizer ######################" << std::endl;
        LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

        std::cout << "###################### begin optimize ######################" << std::endl;
        Values result = optimizer.optimize();

        std::vector<Quadrotor::State> opt_trj;
        uint32_t ikey = 0;
        gtsam::Pose3 i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = 0; ikey < 19; ikey++)
        {
                std::cout << "--------------------------------- " << red << ikey << def << " ----------------------------------" << std::endl;
                i_pose = result.at<Pose3>(X(ikey));
                std::cout << green << "OPT Translation: \n"
                        << i_pose.translation() << std::endl;
                gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(t0 + ikey * dt)), circle_generator.pos(t0 + ikey * dt));
                std::cout << "REF Translation: \n"
                        << ref_pose.translation() << std::endl;

                std::cout << "OPT Rotation: \n"
                        << Rot3::Logmap(i_pose.rotation()).transpose() << std::endl;
                std::cout << "REF Rotation: \n"
                        << Rot3::Logmap(ref_pose.rotation()).transpose() << std::endl;

                vel = result.at<Vector3>(V(ikey));
                std::cout << "OPT VEL: \n"
                        << vel.transpose() << std::endl;
                gtsam::Vector3 ref_vel = circle_generator.vel(t0 + ikey * dt); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                std::cout << "REF VEL: \n"
                        << ref_vel.transpose() << std::endl;

                omega = result.at<Vector3>(S(ikey));
                std::cout << "OPT OMEGA: \n"
                        << omega.transpose() << std::endl;
                gtsam::Vector3 ref_omega = circle_generator.omega(t0 + ikey * dt); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
                std::cout << "REF OMEGA: \n"
                        << ref_omega.transpose() << std::endl;


                input = result.at<gtsam::Vector4>(U(ikey));
                std::cout << "OPT INPUT: \n"
                        << input.transpose() << std::endl;
                std::cout << "REF_INPUT: \n"
                        << circle_generator.inputfm(t0 + ikey * dt).transpose() << std::endl;

        }
        m_state.x = i_pose.translation();
        m_state.rot = i_pose.rotation();
        m_state.v = vel;
        m_state.omega = omega;

        input = result.at<gtsam::Vector4>(U(0));
        state_predicted.force_moment = input;
        
        quad_.setState(state_predicted);
        // quad_.setInput(input[0], input[1], input[2], input[3]);

        input = result.at<gtsam::Vector4>(U(1));
        quad_.stepODE(dt, input);
        // quad_.step_noise(dt);
        quad_.render();
        // std::cout << "RUNNING: \n" << std::endl;
        
        state_predicted = quad_.getState();
        std::cout << "new statex: " << state_predicted.x.transpose() << std::endl;
        std::cout << "new stater: " << Rot3::Logmap(state_predicted.rot).transpose() << std::endl;
        std::cout << "new statev: " << state_predicted.v.transpose() << std::endl;
        std::cout << "new stateomega: " << state_predicted.omega.transpose() << std::endl;
        std::cout << def;

    }

    // quad_.render_test(opt_trj);

    while (true)
    {
        quad_.render();
    }

    return 0;
}
