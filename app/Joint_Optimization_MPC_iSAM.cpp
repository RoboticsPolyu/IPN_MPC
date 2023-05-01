#include "color.h"
#include "trajectory_generator/Trajectory_generator.h"
#include "quadrotor_simulator/Dynamics_control_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"

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
    // Motion generation delta time
    double dt = 0.0001, radius = 1.0, linear_vel = 1.0;
    circle_generator circle_generator(radius, linear_vel, dt);

    // gtsam::LevenbergMarquardtParams parameters;
    // parameters.absoluteErrorTol = 1e-8;
    // parameters.relativeErrorTol = 1e-8;
    // parameters.maxIterations = 500;
    // parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    // parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    parameters.cacheLinearizedFactors = false;
    parameters.enableDetailedResults = true;
    parameters.print();

    ISAM2 isam(parameters);

    auto input_noise = noiseModel::Diagonal::Sigmas(Vector4(2, 1e-3, 1e-3, 1e-3));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(12) << Vector3::Constant(0.005), Vector3::Constant(0.005), Vector3::Constant(0.005), Vector3::Constant(0.005)).finished());
    
    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.001)).finished());
    auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
    auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));

    auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.03)).finished());
    auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(.3, .3, .3));
    auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(.3, .3, .3));

    int opt_lens_traj = 20;

    dt = 0.01; // Model predictive control duration
    Quadrotor quad_;
    Color::Modifier red(Color::FG_RED);
    Color::Modifier def(Color::FG_DEFAULT);
    Color::Modifier green(Color::FG_GREEN);

    Quadrotor::State state_predicted;
    
    NonlinearFactorGraph newFactors;
    Values newValues;

    
    int cur_est_idx = 0;

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
        
        if(traj_idx == 0)
        {
            for (int idx = 0; idx < opt_lens_traj; idx++)
            {
                if (idx == 0)
                {
                    // measurement part state prior
                    newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx), gtsam::Pose3(state_predicted.rot, state_predicted.x), vicon_noise));
                    newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), state_predicted.v, vel_noise));
                    newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx), state_predicted.omega, omega_noise));

                    newValues.insert(X(idx), gtsam::Pose3(state_predicted.rot, state_predicted.x));
                    newValues.insert(V(idx), state_predicted.v);
                    newValues.insert(S(idx), state_predicted.omega);
                }

                DynamicsFactorTm dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1), V(idx + 1), S(idx + 1), dt, dynamics_noise);
                newFactors.add(dynamics_factor);
                
                gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + (idx + 1) * dt)), circle_generator.pos(t0 + (idx + 1) * dt));
                gtsam::Vector3 vel_idx = circle_generator.vel(t0 + (idx + 1) * dt);
                gtsam::Vector3 omega_idx = circle_generator.omega(t0 + (idx + 1) * dt);

                newValues.insert(X(idx + 1), pose_idx);
                newValues.insert(V(idx + 1), vel_idx);
                newValues.insert(S(idx + 1), omega_idx);
                
                gtsam::Vector4 init_input = circle_generator.inputfm(t0 + idx * dt);

                // newFactors.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
                newValues.insert(U(idx), init_input);
                
                // control part state prior
                newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
                newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
                newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));

                // FactorIndices contains there initial state prior factor (3), dynamics factor (1) + control state prior (3) 
                // FactorIndices sum : 4 * 20 + 3 = 83

            }

        }
        else
        {
            /* Add new control target state and its constraints */
            int idx = cur_est_idx + opt_lens_traj - 1;

            DynamicsFactorTm dynamics_factor(X(idx), V(idx), S(idx), U(idx), X(idx + 1), V(idx + 1), S(idx + 1), dt, dynamics_noise);
            newFactors.add(dynamics_factor);
            
            gtsam::Pose3 pose_idx(gtsam::Rot3::Expmap(circle_generator.theta(t0 + (idx + 1) * dt)), circle_generator.pos(t0 + (idx + 1) * dt));
            gtsam::Vector3 vel_idx = circle_generator.vel(t0 + (idx + 1) * dt);
            gtsam::Vector3 omega_idx = circle_generator.omega(t0 + (idx + 1) * dt);

            newValues.insert(X(idx + 1), pose_idx);
            newValues.insert(V(idx + 1), vel_idx);
            newValues.insert(S(idx + 1), omega_idx);
            
            gtsam::Vector4 init_input = circle_generator.inputfm(t0 + idx * dt);

            // newFactors.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), init_input, input_noise));
            newValues.insert(U(idx), init_input);

            newFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
            newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
            newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(S(idx + 1), omega_idx, ref_predict_omega_noise));

            /* Add current state measurement */
            std::cout << "Add current state measurement: " << cur_est_idx << std::endl;
            newFactors.addPrior(X(cur_est_idx), gtsam::Pose3(state_predicted.rot, state_predicted.x), vicon_noise);
            newFactors.addPrior(V(cur_est_idx), state_predicted.v, vel_noise);
            newFactors.addPrior(S(cur_est_idx), state_predicted.omega, omega_noise);

            // FactorIndices: adding newest control state prior (3), then current estimation state prior (3)
            // FactorIndices = 83 + (3 + 3) * cur_est_idx 
        }

        ISAM2Result resultI = isam.update(newFactors, newValues);
        resultI.print("ISAM2 update: ");
        Values result = isam.calculateEstimate();
        result.print("Current estimate: ");

        std::vector<Quadrotor::State> opt_trj;
        gtsam::Pose3 i_pose;
        gtsam::Vector3 vel;
        gtsam::Vector3 omega;
        gtsam::Vector4 input;

        for (uint32_t ikey = cur_est_idx; ikey < opt_lens_traj + cur_est_idx; ikey++)
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

            if(ikey != opt_lens_traj - 1)
            {
                    input = result.at<gtsam::Vector4>(U(ikey));
                    std::cout << "OPT INPUT: \n"
                            << input.transpose() << std::endl;
                    std::cout << "REF_INPUT: \n"
                            << circle_generator.inputfm(t0 + ikey * dt).transpose() << std::endl;
            }
            Quadrotor::State m_state;
            m_state.x = i_pose.translation();
            opt_trj.push_back(m_state);

        }

        input = result.at<gtsam::Vector4>(U(cur_est_idx));
        state_predicted.force_moment = input;
        
        quad_.setState(state_predicted);

        input = result.at<gtsam::Vector4>(U(cur_est_idx + 1));
        quad_.stepODE(dt, input);

        gtsam::Vector3 err;
        err.Zero();
        quad_.renderHistoryOpt(opt_trj, err);
        state_predicted = quad_.getState();

        FactorIndices marginalFactorsIndices;
        FactorIndices deletedFactorsIndices;
        if(cur_est_idx <= opt_lens_traj)
        {
            deletedFactorsIndices.push_back(0 + cur_est_idx* 4);
            deletedFactorsIndices.push_back(1 + cur_est_idx* 4);
            deletedFactorsIndices.push_back(2 + cur_est_idx* 4);
            // deletedFactorsIndices.push_back(3 + cur_est_idx* 4);
        }
        else
        {
        //     deletedFactorsIndices.push_back(); 
        //     deletedFactorsIndices.push_back(); 
        //     deletedFactorsIndices.push_back();    
        }

        cur_est_idx++;
        FastList<Key> leafKeys;
        leafKeys.push_back(X(cur_est_idx - 1));
        leafKeys.push_back(V(cur_est_idx - 1));
        leafKeys.push_back(S(cur_est_idx - 1));
        leafKeys.push_back(U(cur_est_idx - 1));

        isam.marginalizeLeaves(leafKeys, marginalFactorsIndices, deletedFactorsIndices);

        newFactors.resize(0);
        newValues.clear();
    }

    while (true)
    {
        quad_.renderHistoryTrj();
    }

    return 0;
}
