#include "control/Minimum_snap_controller.h"


namespace Control
{
    MiniSnapController::MiniSnapController(/* args */)
    {
        YAML::Node       config = YAML::LoadFile("../config/se3_controller.yaml");  
        controller_params_.kep = config["KeP"].as<double>();  
        controller_params_.kev = config["KeV"].as<double>();
        controller_params_.ker = config["KeR"].as<double>(); 
        controller_params_.kew = config["KeW"].as<double>();
    }

    MiniSnapController::~MiniSnapController()
    {

    }

    MiniSnapController::FeedOut & MiniSnapController::Feedback(const Quadrotor::State &est_state, const Quadrotor::State &des_state)
    {
        MiniSnapController::FeedOut feed_output;

        gtsam::Vector3 zw(0, 0, 1);
        gtsam::Vector3 eP = des_state.p - est_state.p;
        gtsam::Vector3 eV = des_state.v - est_state.v;
        feed_output.acc_z = est_state.rot.unrotate(controller_params_.kep * eP + controller_params_.kev * eV + g_ * zw).z() + des_state.acc_z;

        gtsam::Vector3 eR = gtsam::Rot3::Logmap(est_state.rot.between(des_state.rot));
        gtsam::Vector3 eW = est_state.body_rate - des_state.rot.matrix().transpose()* est_state.rot.matrix() * des_state.body_rate;

        feed_output.body_rate =  controller_params_.ker * eR + des_state.body_rate;
        // feed_output.moment = -controller_params_.ker * eR - controller_params_.kew * eW + gtsam::Vector3(des_state.thrust_torque[1], des_state.thrust_torque[2], des_state.thrust_torque[3]);

        return feed_output;
    }
} // namespace Control
