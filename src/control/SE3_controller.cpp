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

        gtsam::Vector3 eP = cur_state.x - target_state.x;
        gtsam::Vector3 eV = cur_state.v - target_state.v;
        gtsam::Vector3 eR = gtsam::Rot3::Logmap(cur_state.rot.between(target_state.rot));
        gtsam::Vector3 eW = cur_state.omega - target_state.rot.matrix().transpose()* cur_state.rot.matrix() * target_state.omega;

        feed_output.force = (-se3_params_.kep * eP + se3_params_.kev * eV).transpose() * cur_state.rot.matrix() * gtsam::Vector3(0, 0, 1) + target_state.force_moment[0];
        feed_output.moment = -se3_params_.ker * eR + -se3_params_.kew * eW + gtsam::Vector3(target_state.force_moment[1], target_state.force_moment[2], target_state.force_moment[3]);

        return feed_output;
    }
} // namespace Control
