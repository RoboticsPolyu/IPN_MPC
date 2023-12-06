#ifndef __MINI_SNAP_CONTROLLER__
#define __MINI_SNAP_CONTROLLER__

#include "quadrotor_simulator/Dynamics_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <yaml-cpp/yaml.h>

/**
 * Cited Minimum Snap Trajectory Generation and Control for Quadrotors
*/

using namespace QuadrotorSim_SO3;
using namespace UAVFactor;

namespace Control
{
    class MiniSnapController
    {

    public:
        struct SE3_Params
        {
            double kep;
            double kev;
            double ker;
            double kew;
        };

        struct FeedOut
        {
            double         acc_z;
            gtsam::Vector3 body_rate;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

        MiniSnapController(/* args */);

        FeedOut & Feedback(const Quadrotor::State &est_state, const Quadrotor::State &des_state);

        ~MiniSnapController();    
        
    private:
        MiniSnapController::SE3_Params controller_params_;
        DynamicsParams dynamics_params_;

        const float g_ = 9.81;

    };

} // namespace Control

#endif // __MINI_SNAP_CONTROLLER__