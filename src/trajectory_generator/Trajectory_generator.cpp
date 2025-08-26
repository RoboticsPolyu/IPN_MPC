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
        double yaw_ = - angular_speed * t;
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
        return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t - dt_)).between(gtsam::Rot3::Expmap(theta(t + dt_)))) / 2 / dt_;
    }

    gtsam::Vector3 circle_generator::thrust(double t)
    {
        gtsam::Vector force;
        double angular_speed = speed_ / radius_;
        force = gtsam::Vector3(-sin(angular_speed * t) * angular_speed * angular_speed * radius_,
                               -cos(angular_speed * t) * angular_speed * angular_speed * radius_,
                               0) 
                               + g_;

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

        double dx0 = 0.10;
        double dx1 = 0.10;

        K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
              dx0 * dynamics_params_.k_f, - dx0 * dynamics_params_.k_f, - dx0 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,
            - dx1 * dynamics_params_.k_f, - dx1 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,
              dynamics_params_.k_m,       - dynamics_params_.k_m,         dynamics_params_.k_m,        - dynamics_params_.k_m;
        
        // std::cout << "K1: " << K1 << std::endl; 
        gtsam::Vector4 input;

        input = K1.inverse() * Tmb;
        // std::cout << K1.inverse() << "\n";
        // std::cout << Tmb << " \n";


        input[0] = sqrt(input[0]);
        input[1] = sqrt(input[1]);
        input[2] = sqrt(input[2]);
        input[3] = sqrt(input[3]);
        // std::cout << input << " \n";
        return input;
    }

    
    gtsam::Vector3 vertical_circle_generator::pos(double t)
    {
        double angular_speed = speed_ / radius_; // 2*pi/(round / speed) = 2*pi/(2*pi*r)*speed =
        return gtsam::Vector3(sin(angular_speed * t) * radius_, 0.0, cos(angular_speed * t) * radius_);
    }

    gtsam::Vector3 vertical_circle_generator::vel(double t)
    {
        double angular_speed = speed_ / radius_;

        // double v_x = (sin(angular_speed * (t + dt_)) * radius_ - sin(angular_speed * (t)) * radius_) / (dt_);
        // double v_y = (cos(angular_speed * (t + dt_)) * radius_ - cos(angular_speed * (t)) * radius_) / (dt_);

        gtsam::Vector3 vel = gtsam::Vector3(cos(angular_speed * t) * angular_speed * radius_, 0.0, -sin(angular_speed * t) * angular_speed * radius_);
        return vel;
    }

    gtsam::Vector3 vertical_circle_generator::theta(double t)
    {
        gtsam::Vector3 force;

        force = thrust(t);
        double yaw_ = 0.0;
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

    gtsam::Vector3 vertical_circle_generator::omega(double t)
    {
        // return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t - dt_)).between(gtsam::Rot3::Expmap(theta(t + dt_)) ) ) / (2*dt_);
        return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t - dt_)).between(gtsam::Rot3::Expmap(theta(t + dt_)))) / 2 / dt_;
    }

    gtsam::Vector3 vertical_circle_generator::thrust(double t)
    {
        gtsam::Vector force;
        double angular_speed = speed_ / radius_;
        force = gtsam::Vector3(-sin(angular_speed * t) * angular_speed * angular_speed * radius_,
                               0.0,
                               -cos(angular_speed * t) * angular_speed * angular_speed * radius_) +
                g_;

        return force;
    }

    gtsam::Vector4 vertical_circle_generator::inputfm(double t)
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

    gtsam::Vector4 vertical_circle_generator::input(double t)
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

        double dx0 = 0.10;
        double dx1 = 0.10;

        K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
              dx0 * dynamics_params_.k_f, - dx0 * dynamics_params_.k_f, - dx0 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,
            - dx1 * dynamics_params_.k_f, - dx1 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,
              dynamics_params_.k_m,       - dynamics_params_.k_m,         dynamics_params_.k_m,        - dynamics_params_.k_m;
        
        // std::cout << "K1: " << K1 << std::endl; 
        gtsam::Vector4 input;

        input = K1.inverse() * Tmb;
        // std::cout << K1.inverse() << "\n";
        // std::cout << Tmb << " \n";


        input[0] = sqrt(input[0]);
        input[1] = sqrt(input[1]);
        input[2] = sqrt(input[2]);
        input[3] = sqrt(input[3]);
        // std::cout << input << " \n";
        return input;
    }

    /*Circle motion based on constant linear acc*/

    double cir_conacc_generator::angle(double t)
    {
        double t_shark = max_speed_ / acc_;
        double angular_acc = acc_ * radius_;
        double angle = 0;
        if (t < t_shark)
        {
            angle = 0.5 * angular_acc * t * t;
        }
        else
        {
            angle = 0.5 * angular_acc * t_shark * t_shark + max_speed_ / radius_ * (t - t_shark);
        }

        return angle;
    }

    gtsam::Vector3 cir_conacc_generator::pos(double t)
    {
        double t_shark = max_speed_ / acc_;
        double angular_acc = acc_ * radius_;
        double angle = 0;
        if (t < t_shark)
        {
            angle = 0.5 * angular_acc * t * t;
        }
        else
        {
            angle = 0.5 * angular_acc * t_shark * t_shark + max_speed_ / radius_ * (t - t_shark);
        }

        return gtsam::Vector3(sin(angle) * radius_, cos(angle) * radius_, 1.0);
    }

    gtsam::Vector3 cir_conacc_generator::vel(double t)
    {
        double t_shark = max_speed_ / acc_;
        double angular_acc = acc_ * radius_;
        double angle = 0;
        if (t < t_shark)
        {
            angle = 0.5 * angular_acc * t * t;
        }
        else
        {
            angle = 0.5 * angular_acc * t_shark * t_shark + max_speed_ / radius_ * (t - t_shark);
        }

        gtsam::Vector3 vel = gtsam::Vector3(cos(angle) * angular_acc * t * radius_, -sin(angle) * angular_acc * t * radius_, 0);
        // std::cout << "Analytical vel: " << vel.transpose() << std::endl;

        // vel = (pos(t + dt_) - pos(t)) / dt_; // better than Analytical solution ???
        // std::cout << "Diff vel:     " << vel.transpose() << std::endl;

        return vel;
    }

    gtsam::Vector3 cir_conacc_generator::theta(double t)
    {
        double yaw = - angle(t);

        gtsam::Vector3 force = thrust(t);
        gtsam::Vector3 zB = force / force.norm();
        gtsam::Vector3 xC(cos(yaw), sin(yaw), 0);
        gtsam::Vector3 yB = zB.cross(xC) / zB.cross(xC).norm();
        gtsam::Vector3 xB = yB.cross(zB);

        gtsam::Matrix3 R;
        R.col(0) = xB;
        R.col(1) = yB;
        R.col(2) = zB;
        gtsam::Rot3 rot3(R);
        return gtsam::Rot3::Logmap(rot3);
    }

    gtsam::Vector3 cir_conacc_generator::omega(double t)
    {
        return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t)).between(gtsam::Rot3::Expmap(theta(t + dt_)))) / dt_;
    }

    gtsam::Vector3 cir_conacc_generator::thrust(double t)
    {
        gtsam::Vector force;

        force = (vel(t + dt_) - vel(t - dt_)) / 2 / dt_ + g_;

        return force;
    }

    gtsam::Vector4 cir_conacc_generator::inputfm(double t)
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

        return Tmb;
    }

    gtsam::Vector4 cir_conacc_generator::input(double t)
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

        // K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
        //     0, 0, dynamics_params_.arm_length * dynamics_params_.k_f, -dynamics_params_.arm_length * dynamics_params_.k_f,
        //     -dynamics_params_.arm_length * dynamics_params_.k_f, dynamics_params_.arm_length * dynamics_params_.k_f, 0, 0,
        //     dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m, -dynamics_params_.k_m;

        double dx0 = 0.10;
        double dx1 = 0.10;

        K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
              dx0 * dynamics_params_.k_f, - dx0 * dynamics_params_.k_f, - dx0 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,
            - dx1 * dynamics_params_.k_f, - dx1 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,   dx0 * dynamics_params_.k_f,
              dynamics_params_.k_m,       - dynamics_params_.k_m,         dynamics_params_.k_m,        - dynamics_params_.k_m;

        gtsam::Vector4 input;

        input = K1.inverse() * Tmb;

        input[0] = sqrt(input[0]);
        input[1] = sqrt(input[1]);
        input[2] = sqrt(input[2]);
        input[3] = sqrt(input[3]);

        return input;
    }

    gtsam::Vector3 figure_eight_generator::pos(double t)
    {
        double tau = speed_ * t;  // Scaled time for speed control
        return gtsam::Vector3(
            scale_ * sin(tau),
            scale_ * sin(tau) * cos(tau),
            // 1.0 + scale_* 0.1 * cos(tau)
            1.0
        );
    }

    gtsam::Vector3 figure_eight_generator::vel(double t)
    {
        double tau = speed_ * t;
        double cos_tau = cos(tau);
        double sin_tau = sin(tau);
        return gtsam::Vector3(
            scale_ * speed_ * cos_tau,
            scale_ * speed_ * cos(2 * tau),
            0.0
        );
    }

    // gtsam::Vector3 figure_eight_generator::theta(double t)
    // {
    //     gtsam::Vector3 force = thrust(t);
    //     double yaw = atan2(vel(t)(1), vel(t)(0));  // Yaw aligns with velocity direction
    //     gtsam::Vector3 zB = force / force.norm();
    //     gtsam::Vector3 xC(cos(yaw), sin(yaw), 0);
    //     gtsam::Vector3 yB = zB.cross(xC) / zB.cross(xC).norm();
    //     gtsam::Vector3 xB = yB.cross(zB);

    //     gtsam::Matrix3 R;
    //     R.col(0) = xB;
    //     R.col(1) = yB;
    //     R.col(2) = zB;
    //     gtsam::Rot3 rot3(R);
    //     return gtsam::Rot3::Logmap(rot3);
    // }

    gtsam::Vector3 figure_eight_generator::theta(double t)
    {
        double         yaw   = 0.0f;
        gtsam::Vector3 force = thrust(t);
        gtsam::Vector3 zB    = force / force.norm();
        gtsam::Vector3 xC(cos(yaw), sin(yaw), 0);
        gtsam::Vector3 yB    = zB.cross(xC) / zB.cross(xC).norm();
        gtsam::Vector3 xB    = yB.cross(zB);

        gtsam::Matrix3 R;
        R.col(0) = xB;
        R.col(1) = yB;
        R.col(2) = zB;
        gtsam::Rot3 rot3(R);
        return gtsam::Rot3::Logmap(rot3);
    }

    gtsam::Vector3 figure_eight_generator::omega(double t)
    {
        return gtsam::Rot3::Logmap(
            gtsam::Rot3::Expmap(theta(t - dt_)).between(gtsam::Rot3::Expmap(theta(t + dt_)))
        ) / (2 * dt_);
    }

    gtsam::Vector3 figure_eight_generator::thrust(double t)
    {
        // Numerical acceleration using central difference
        gtsam::Vector3 acc = (vel(t + dt_) - vel(t - dt_)) / (2 * dt_);
        return acc + g_;
    }

    gtsam::Vector4 figure_eight_generator::inputfm(double t)
    {
        gtsam::Vector3 acc = thrust(t);
        gtsam::Vector3 d_omega = (omega(t + dt_) - omega(t)) / dt_;

        gtsam::Matrix33 J;
        J << dynamics_params_.Ixx, 0, 0,
             0, dynamics_params_.Iyy, 0,
             0, 0, dynamics_params_.Izz;

        gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));
        gtsam::Vector4 Tmb(acc.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));
        return Tmb;
    }

    gtsam::Vector4 figure_eight_generator::input(double t)
    {
        gtsam::Vector3 force = thrust(t);
        gtsam::Vector3 d_omega = (omega(t + dt_) - omega(t)) / dt_;

        gtsam::Matrix33 J;
        J << dynamics_params_.Ixx, 0, 0,
             0, dynamics_params_.Iyy, 0,
             0, 0, dynamics_params_.Izz;

        gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));
        gtsam::Vector4 Tmb(force.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));

        gtsam::Matrix4 K1;
        double dx0 = 0.10;
        double dx1 = 0.10;

        K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
              dx0 * dynamics_params_.k_f, -dx0 * dynamics_params_.k_f, -dx0 * dynamics_params_.k_f, dx0 * dynamics_params_.k_f,
              -dx1 * dynamics_params_.k_f, -dx1 * dynamics_params_.k_f, dx0 * dynamics_params_.k_f, dx0 * dynamics_params_.k_f,
              dynamics_params_.k_m, -dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m;

        gtsam::Vector4 input = K1.inverse() * Tmb;

        // Ensure non-negative inputs for motor speeds
        input[0] = sqrt(std::max(0.0, input[0]));
        input[1] = sqrt(std::max(0.0, input[1]));
        input[2] = sqrt(std::max(0.0, input[2]));
        input[3] = sqrt(std::max(0.0, input[3]));

        return input;
    }

    void back_and_forth_generator::get_segment_state(double t, int& direction, 
                                                double& normalized_time, 
                                                double& s, double& v, double& a)
    {
        // 计算当前周期编号和周期内的时间
        double total_cycle_time = 2 * one_way_time_;
        int cycle_count = static_cast<int>(t / total_cycle_time);
        double cycle_time = fmod(t, total_cycle_time);
        
        // 确定方向：前半周期正向，后半周期反向
        direction = (cycle_time < one_way_time_) ? 1 : -1;
        
        // 计算在当前周期内的时间（归一化到0到one_way_time_之间）
        normalized_time = (cycle_time < one_way_time_) ? cycle_time : cycle_time - one_way_time_;
        
        // 根据时间段计算位置、速度、加速度
        if (normalized_time < acc_time_)
        {
            // 加速段（正向或反向的第一阶段）
            s = 0.5 * max_acc_ * normalized_time * normalized_time;
            v = max_acc_ * normalized_time;
            a = max_acc_;
        }
        else if (normalized_time < acc_time_ + cruise_time_)
        {
            // 匀速段
            double cruise_start_time = normalized_time - acc_time_;
            s = acc_distance_ + max_speed_ * cruise_start_time;
            v = max_speed_;
            a = 0.0;
        }
        else if (normalized_time < one_way_time_)
        {
            // 减速段
            double decel_start_time = normalized_time - acc_time_ - cruise_time_;
            double decel_time = one_way_time_ - acc_time_ - cruise_time_;
            s = length_ - 0.5 * max_acc_ * (decel_time - decel_start_time) * (decel_time - decel_start_time);
            v = max_speed_ - max_acc_ * decel_start_time;
            a = -max_acc_;
        }
        else
        {
            // 端点，速度为零
            s = length_;
            v = 0.0;
            a = 0.0;
        }
        
        // 如果是反向运动，调整位置和速度的方向
        if (direction == -1)
        {
            s = length_ - s;
            v = -v;
            a = -a;
        }
    }

    gtsam::Vector3 back_and_forth_generator::pos(double t)
    {
        int direction;
        double normalized_time, s, v, a;
        get_segment_state(t, direction, normalized_time, s, v, a);
        
        // 假设直线沿x轴方向，从原点开始往返运动
        return gtsam::Vector3(s + x_offset_, 0.0 + y_offset_, 1.0 + z_offset_);
    }

    gtsam::Vector3 back_and_forth_generator::vel(double t)
    {
        int direction;
        double normalized_time, s, v, a;
        get_segment_state(t, direction, normalized_time, s, v, a);
        
        return gtsam::Vector3(v, 0.0, 0.0);
    }

    gtsam::Vector3 back_and_forth_generator::theta(double t)
    {
        // 对于直线运动，姿态主要保持水平
        // 可以根据速度方向调整偏航角，这里保持为零
        // double yaw = 0.0;
        
        // gtsam::Vector3 force = thrust(t);
        // if (force.norm() < 1e-6)
        // {
        //     // 避免除以零，返回默认姿态
        //     return gtsam::Vector3::Zero();
        // }
        
        // gtsam::Vector3 zB = force / force.norm();
        // gtsam::Vector3 xC(cos(yaw), sin(yaw), 0);
        
        // // 检查叉积是否为零向量
        // gtsam::Vector3 cross_product = zB.cross(xC);
        // if (cross_product.norm() < 1e-6)
        // {
        //     // 如果叉积接近零，使用默认的y轴
        //     gtsam::Vector3 yB(0, 1, 0);
        //     gtsam::Vector3 xB = yB.cross(zB);
            
        //     gtsam::Matrix3 R;
        //     R.col(0) = xB.normalized();
        //     R.col(1) = yB.normalized();
        //     R.col(2) = zB.normalized();
        //     gtsam::Rot3 rot3(R);
        //     return gtsam::Rot3::Logmap(rot3);
        // }
        
        // gtsam::Vector3 yB = cross_product / cross_product.norm();
        // gtsam::Vector3 xB = yB.cross(zB);

        // gtsam::Matrix3 R;
        // R.col(0) = xB.normalized();
        // R.col(1) = yB.normalized();
        // R.col(2) = zB.normalized();
        // gtsam::Rot3 rot3(R);
        // return gtsam::Rot3::Logmap(rot3);

        gtsam::Vector3 force;
        force = thrust(t);
        double yaw_ = 0;
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

    gtsam::Vector3 back_and_forth_generator::omega(double t)
    {
        // 使用中心差分法计算角速度，更精确
        if (t < dt_ || t > one_way_time_ * 2 - dt_)
        {
            // 在时间边界处返回零角速度
            return gtsam::Vector3::Zero();
        }
        
        gtsam::Vector3 theta_prev = theta(t - dt_);
        gtsam::Vector3 theta_next = theta(t + dt_);
        return (theta_next - theta_prev) / (2 * dt_);
    }

    gtsam::Vector3 back_and_forth_generator::thrust(double t)
    {
        int direction;
        double normalized_time, s, v, a;
        get_segment_state(t, direction, normalized_time, s, v, a);
        
        // 推力需要抵消重力和提供前进加速度
        gtsam::Vector3 linear_acc = gtsam::Vector3(a, 0.0, 0.0);
        return linear_acc + g_;
    }

    gtsam::Vector4 back_and_forth_generator::inputfm(double t)
    {
        gtsam::Vector3 acc = thrust(t);
        if (t < dt_ || t > one_way_time_ * 2 - dt_)
        {
            // 在时间边界处使用前向或后向差分
            gtsam::Vector3 d_omega;
            if (t < dt_)
            {
                d_omega = (omega(t + dt_) - omega(t)) / dt_;
            }
            else
            {
                d_omega = (omega(t) - omega(t - dt_)) / dt_;
            }
            
            gtsam::Matrix33 J;
            J << dynamics_params_.Ixx, 0, 0,
                 0, dynamics_params_.Iyy, 0,
                 0, 0, dynamics_params_.Izz;

            gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));
            gtsam::Vector4 Tmb(acc.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));
            return Tmb;
        }
        
        gtsam::Vector3 d_omega = (omega(t + dt_) - omega(t - dt_)) / (2 * dt_);

        gtsam::Matrix33 J;
        J << dynamics_params_.Ixx, 0, 0,
             0, dynamics_params_.Iyy, 0,
             0, 0, dynamics_params_.Izz;

        gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));
        gtsam::Vector4 Tmb(acc.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));
        return Tmb;
    }

    gtsam::Vector4 back_and_forth_generator::input(double t)
    {
        gtsam::Vector3 force = thrust(t);
        gtsam::Vector3 d_omega;
        
        if (t < dt_ || t > one_way_time_ * 2 - dt_)
        {
            // 在时间边界处使用前向或后向差分
            if (t < dt_)
            {
                d_omega = (omega(t + dt_) - omega(t)) / dt_;
            }
            else
            {
                d_omega = (omega(t) - omega(t - dt_)) / dt_;
            }
        }
        else
        {
            // 使用中心差分法
            d_omega = (omega(t + dt_) - omega(t - dt_)) / (2 * dt_);
        }

        gtsam::Matrix33 J;
        J << dynamics_params_.Ixx, 0, 0,
             0, dynamics_params_.Iyy, 0,
             0, 0, dynamics_params_.Izz;

        gtsam::Vector3 mb = J * d_omega + omega(t).cross(J * omega(t));
        gtsam::Vector4 Tmb(force.norm() * dynamics_params_.mass, mb(0), mb(1), mb(2));

        gtsam::Matrix4 K1;
        double dx0 = 0.10;
        double dx1 = 0.10;

        K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
              dx0 * dynamics_params_.k_f, -dx0 * dynamics_params_.k_f, -dx0 * dynamics_params_.k_f, dx0 * dynamics_params_.k_f,
              -dx1 * dynamics_params_.k_f, -dx1 * dynamics_params_.k_f, dx0 * dynamics_params_.k_f, dx0 * dynamics_params_.k_f,
              dynamics_params_.k_m, -dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m;

        gtsam::Vector4 input = K1.inverse() * Tmb;

        // 确保电机速度为非负
        input[0] = sqrt(std::max(0.0, input[0]));
        input[1] = sqrt(std::max(0.0, input[1]));
        input[2] = sqrt(std::max(0.0, input[2]));
        input[3] = sqrt(std::max(0.0, input[3]));

        return input;
    }
}