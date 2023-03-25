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

using symbol_shorthand::U;
using symbol_shorthand::X;

int main(void)
{

    simple_trajectory_generator stg(gtsam::Vector3(0.1, 0.1, 0.05), 0);

    std::vector<Quadrotor::State> reference_of_states;

    /*generate a simple line trajectory*/
    for (int t = 0; t < 20; t = t + 1)
    {
        std::cout << stg.input(t * 0.10) << std::endl;
    }

    std::cout << "simple_trajectory_generator_test" << std::endl;

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
    float dt = 0.01;

    for (int idx = 0; idx < reference_of_states.size() - opt_lens_traj; idx++)
    {
        DynamicsFactor2 dynamics_factor2(X(idx), U(idx), X(idx + 1), 0.01f, dynamics_noise);
        graph.add(dynamics_factor2);

        gtsam::Vector12 _state;
        gtsam::Vector4 _input; // reference_of_states[idx]
        if (idx != 0)
        {
            graph.add(gtsam::PriorFactor<gtsam::Vector12>(X(idx), _state, dynamics_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector4>(U(idx), _input, input_noise));
        }

        if (idx == 19)
        {
        }
    }

    return 0;
}

// auto pos_noise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
// auto vel_noise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
// auto theta_noise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
// auto omega_noise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
