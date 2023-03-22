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

using namespace std;
using namespace gtsam;
using namespace uav_factor;
using namespace Trajectory;

using symbol_shorthand::U;
using symbol_shorthand::X;

int main(void)
{
    simple_trajectory_generator stg(gtsam::Vector3(0.1, 0.1, 0.05), 0);

    /*generate a simple line trajectory*/
    for(int t = 0; t < 20; t = t +1)
    {
        std::cout << stg.input(t* 0.10) << std::endl;
    }

    std::cout << "simple_trajectory_generator_test" << std::endl;
    return 0;
}