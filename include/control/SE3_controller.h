#ifndef __SE3_CONTROLLER_H__
#define __SE3_CONTROLLER_H__

#include "quadrotor_simulator/Dynamics_factor.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <yaml-cpp/yaml.h>


using namespace QuadrotorSim_SO3;
using namespace UAV_Factor;

namespace Control
{
    class SE3_controller
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
            double force;
            gtsam::Vector3 moment;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

        SE3_controller(/* args */);

        FeedOut Feedback(const Quadrotor::State &cur_state, const Quadrotor::State &target_state);

        ~SE3_controller();    
        
    private:
        SE3_controller::SE3_Params se3_params_;
        DynamicsParams dynamics_params_;
    };

} // namespace Control

#endif