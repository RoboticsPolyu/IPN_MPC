#include "control/SE3_controller.h"

namespace Control
{
    SE3_controller::SE3_controller(/* args */)
    {
    }
    
    SE3_controller::~SE3_controller()
    {
    }  

    SE3_controller::FeedOut SE3_controller::Feedback(const Quadrotor::State &cur_state, const Quadrotor::State &target_state)
    {
        SE3_controller::FeedOut feed_output;

        feed_output.force = (-se3_params_.kep* (cur_state.x - target_state.x) +
            se3_params_.kev* (cur_state.v - target_state.v)).transpose() * cur_state.rot.matrix() * gtsam::Vector3(0,0,1) + 
            target_state.force_moment[0];
        //gtsam::Vector3 eR = 0.5*(ta)
        //feed_output.moment = -se3_params_.kr* ()
        return feed_output;
    }
} // namespace Control

