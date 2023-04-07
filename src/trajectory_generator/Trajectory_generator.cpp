#include "trajectory_generator/Trajectory_generator.h"

namespace Trajectory
{
    gtsam::Vector3 circle_generator::pos(double t)
    {
        double angular_speed = speed_ / radius_; // 2*pi/(round / speed) = 2*pi/(2*pi*r)*speed =
        return gtsam::Vector3(sin(angular_speed * t) * radius_, cos(angular_speed * t) * radius_, 1.0);
    }

    gtsam::Vector3 circle_generator::vel(double t)
    {
        double angular_speed = speed_ / radius_;

        // double v_x = (sin(angular_speed * (t + dt_)) * radius_ - sin(angular_speed * (t)) * radius_) / (dt_);
        // double v_y = (cos(angular_speed * (t + dt_)) * radius_ - cos(angular_speed * (t)) * radius_) / (dt_);

        gtsam::Vector3 vel = gtsam::Vector3(cos(angular_speed * t) * angular_speed * radius_, -sin(angular_speed * t) * angular_speed * radius_, 0);
        return vel;
    }

    gtsam::Vector3 circle_generator::theta(double t)
    {
        gtsam::Vector3 force;
        double angular_speed = speed_ / radius_;

        force = thrust(t);
        double yaw_ = -angular_speed * t;
        gtsam::Vector3 zB = force / force.norm();
        gtsam::Vector3 xC(cos(yaw_), sin(yaw_), 0);
        gtsam::Vector3 yB = zB.cross(xC) / zB.cross(xC).norm();
        gtsam::Vector3 xB = yB.cross(zB);

        gtsam::Matrix3 R;
        R.col(0) = xB;
        R.col(1) = yB;
        R.col(2) = zB;
        gtsam::Rot3 rot3(R);
        return gtsam::Rot3::Logmap(rot3);
    }

    gtsam::Vector3 circle_generator::omega(double t)
    {
        // return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t - dt_)).between(gtsam::Rot3::Expmap(theta(t + dt_)) ) ) / (2*dt_);
        return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t)).between(gtsam::Rot3::Expmap(theta(t + dt_)))) / dt_;
    }

    gtsam::Vector3 circle_generator::thrust(double t)
    {
        gtsam::Vector force;
        double angular_speed = speed_ / radius_;
        force = gtsam::Vector3(-sin(angular_speed * t) * angular_speed * angular_speed * radius_,
                               -cos(angular_speed * t) * angular_speed * angular_speed * radius_,
                               0) +
                g_;

        return force;
    }

    gtsam::Vector4 circle_generator::inputfm(double t)
    {
        gtsam::Vector3 acc;
        acc = thrust(t);

        gtsam::Vector3 d_omega = (omega(t + dt_) - omega(t)) / dt_;
        gtsam::Matrix33 J_inv;
        J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
            0, 1.0 / dynamics_params_.Iyy, 0,
            0, 0, 1.0 / dynamics_params_.Izz;
        gtsam::Matrix33 J;
        J << dynamics_params_.Ixx, 0, 0,
            0, dynamics_params_.Iyy, 0,
            0, 0, dynamics_params_.Izz;

        gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));

        gtsam::Vector4 Tmb(acc.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));
        // std::cout << "Tmb: " << Tmb.transpose() << std::endl;
        return Tmb;
    }

    gtsam::Vector4 circle_generator::input(double t)
    {
        gtsam::Vector3 force;
        force = thrust(t);

        gtsam::Vector3 d_omega = (omega(t + dt_) - omega(t)) / dt_;
        gtsam::Matrix33 J_inv;
        J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
            0, 1.0 / dynamics_params_.Iyy, 0,
            0, 0, 1.0 / dynamics_params_.Izz;
        gtsam::Matrix33 J;
        J << dynamics_params_.Ixx, 0, 0,
            0, dynamics_params_.Iyy, 0,
            0, 0, dynamics_params_.Izz;

        gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));
        gtsam::Vector4 Tmb(force.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));

        gtsam::Matrix4 K1;

        K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
            0, 0, dynamics_params_.arm_length * dynamics_params_.k_f, -dynamics_params_.arm_length * dynamics_params_.k_f,
            -dynamics_params_.arm_length * dynamics_params_.k_f, dynamics_params_.arm_length * dynamics_params_.k_f, 0, 0,
            dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m, -dynamics_params_.k_m;
        gtsam::Vector4 input;

        input = K1.inverse() * Tmb;

        input[0] = sqrt(input[0]);
        input[1] = sqrt(input[1]);
        input[2] = sqrt(input[2]);
        input[3] = sqrt(input[3]);

        return input;
    }
}