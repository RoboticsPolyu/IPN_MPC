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
using namespace UAV_Factor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(void)
{
    double dt = 0.001, radius = 1.0, linear_vel = 1.0;
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

    NonlinearFactorGraph graph;
    Values initial_value;

    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(2000, 2000, 2000, 2000));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.001), Vector3::Constant(0.01), Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.001)).finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(0.03, 0.03, 0.03));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.03, 0.03, 0.03));

    auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 1.0));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 1.0));

    int opt_lens_traj = 20;

    dt = 0.01;
    Quadrotor quad_;
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier green(Color::FG_GREEN);

    for (int idx = 0; idx < opt_lens_traj; idx++)
    {
        DynamicsFactor dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1), V(idx + 1), S(idx + 1), dt, dynamics_noise);

        graph.add(dynamics_factor);
        gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta((idx + 1) * dt)), circle_generator.pos((idx + 1) * dt));
        initial_value.insert(X(idx + 1), ref_pose);
        initial_value.insert(V(idx + 1), circle_generator.vel((idx + 1) * dt));
        initial_value.insert(S(idx + 1), circle_generator.omega((idx + 1) * dt));
        gtsam::Vector4 diff_input;
        diff_input.setZero();
        // diff_input << 20, -30, 5, 13;
        gtsam::Vector4 init_input = circle_generator.input(idx * dt) + diff_input;
        initial_value.insert(U(idx), init_input);

        graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));

        gtsam::Pose3 init_pose(gtsam::Rot3::Expmap(circle_generator.theta(idx * dt)), circle_generator.pos(idx * dt));
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), init_pose, ref_predict_pose_noise));
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), circle_generator.vel((idx + 1) * dt), ref_predict_vel_noise));
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), circle_generator.omega((idx + 1) * dt), ref_predict_omega_noise));

        gtsam::Vector4 _input;
        if (idx == 0)
        {
            m_state.x = circle_generator.pos(idx * dt) + gtsam::Vector3(0.0, 0.0, -0.10);
            m_state.v = circle_generator.vel(idx * dt);
            m_state.rot = gtsam::Rot3::Expmap(circle_generator.theta(idx * dt));
            m_state.omega = circle_generator.omega(idx * dt);
            gtsam::Vector4 input = circle_generator.input(idx * dt);
            m_state.motor_rpm << input[0], input[1], input[2], input[3];

            // quad_.setState(m_state);
            // quad_.setInput(input[0] + 500, input[1]+500, input[2] + 500, input[3] + 500);
            // quad_.step(0.02);
            // m_state = quad_.getState();

            gtsam::Pose3 ref_pose(m_state.rot, m_state.x);
            std::cout << "init statex: " << m_state.x.transpose() << std::endl;
            std::cout << "init stater: " << Rot3::Logmap(m_state.rot).transpose() << std::endl;
            std::cout << "init statev: " << m_state.v.transpose() << std::endl;
            std::cout << "init stateomega: " << m_state.omega.transpose() << std::endl;

            graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), ref_pose, vicon_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), m_state.v, vel_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), m_state.omega, omega_noise));

            initial_value.insert(X(idx), ref_pose);
            initial_value.insert(V(idx), circle_generator.vel(idx * dt));
            initial_value.insert(S(idx), circle_generator.omega(idx * dt));
        }
        else
        {
            // graph.add(gtsam::BetweenFactor<gtsam::Vector4>(U(idx - 1), U(idx), gtsam::Vector4::Zero(), input_noise));
        }
    }

    std::cout << "###################### init optimizer ######################" << std::endl;
    LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

    std::cout << "###################### begin optimize ######################" << std::endl;
    Values result = optimizer.optimize();

    std::vector<Quadrotor::State> opt_trj;

    for (uint32_t ikey = 0; ikey < 20; ikey++)
    {

        std::cout << "--------------------------------- " << red << ikey << def << " ----------------------------------" << std::endl;
        Pose3 i_pose = result.at<Pose3>(X(ikey));
        std::cout << green << "OPT Translation: \n"
                  << i_pose.translation() << std::endl;
        gtsam::Pose3 ref_pose(gtsam::Rot3::Expmap(circle_generator.theta(ikey * dt)), circle_generator.pos(ikey * dt));
        std::cout << "REF Translation: \n"
                  << ref_pose.translation() << std::endl;

        gtsam::Vector3 vel = result.at<Vector3>(V(ikey));
        std::cout << "OPT VEL: \n"
                  << vel.transpose() << std::endl;
        gtsam::Vector3 ref_vel = circle_generator.vel(ikey * dt); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
        std::cout << "REF VEL: \n"
                  << ref_vel.transpose() << std::endl;

        gtsam::Vector3 omega = result.at<Vector3>(S(ikey));
        std::cout << "OPT OMEGA: \n"
                  << omega.transpose() << std::endl;
        gtsam::Vector3 ref_omega = circle_generator.omega(ikey * dt); //(gtsam::Rot3::Expmap(circle_generator.theta(ikey* dt)), circle_generator.pos(ikey * dt));
        std::cout << "REF OMEGA: \n"
                  << ref_omega.transpose() << std::endl;

        Quadrotor::State state_;
        state_.x = i_pose.translation();
        state_.rot = i_pose.rotation();
        quad_.setState(state_);
        quad_.render_history_trj();

        state_.x = ref_pose.translation();
        state_.rot = ref_pose.rotation();
        opt_trj.push_back(state_);

        if (ikey != 20)
        {
            gtsam::Vector4 input = result.at<gtsam::Vector4>(U(ikey));
            std::cout << "OPT INPUT: \n"
                      << input.transpose() << std::endl;
            std::cout << "REF_INPUT: \n"
                      << circle_generator.input(ikey * dt).transpose() << std::endl;

            if (ikey == 0)
            {
                
                m_state.x = i_pose.translation();
                m_state.rot = i_pose.rotation();
                m_state.v = vel;
                m_state.omega = omega;
                
                quad_.setState(m_state);
                quad_.setInput(input[0], input[1], input[2], input[3]);
                quad_.step(dt);

                std::cout << "RUNNING: \n"
                          << std::endl;
                m_state = quad_.getState();
                std::cout << "new statex: " << m_state.x.transpose() << std::endl;
                std::cout << "new stater: " << Rot3::Logmap(m_state.rot).transpose() << std::endl;
                std::cout << "new statev: " << m_state.v.transpose() << std::endl;
                std::cout << "new stateomega: " << m_state.omega.transpose() << std::endl;
            }
        }
        std::cout << def;
    }
    gtsam::Vector3 err;
    err.Zero();

    quad_.render_history_opt(opt_trj, err);

    while (true)
    {
        quad_.render_history_trj();
    }

    return 0;
}
