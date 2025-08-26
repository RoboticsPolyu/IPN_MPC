#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <Eigen/Core>
#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_params.h"

using namespace UAVFactor;

typedef struct traj_state
{
    double            t;
    gtsam::Vector3    pos;
    gtsam::Vector3    vel;
    gtsam::Quaternion rotation;
    gtsam::Vector3    angular_speed;
    gtsam::Vector3    acc;
    gtsam::Vector3    a_rot;
    gtsam::Vector4    motor;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}traj_state;

namespace Trajectory
{
    template <class T>
    class Path
    {
    public:
        Path();

        T generate(double time);

    private:
    };

    class back_and_forth_generator
    {
    public:
        back_and_forth_generator(double length, double max_speed, double dt)
            : length_(length),
            max_speed_(max_speed),
            dt_(dt)
        {
            // 计算达到最大速度所需的时间和距离
            max_acc_ = 2.0; // 设置合适的加速度值
            acc_time_ = max_speed_ / max_acc_;
            acc_distance_ = 0.5 * max_acc_ * acc_time_ * acc_time_;
            
            // 检查是否能够达到最大速度
            if (acc_distance_ * 2 > length_)
            {
                // 无法达到最大速度，需要调整加速度和最大速度
                max_acc_ = (max_speed_ * max_speed_) / length_;
                acc_time_ = max_speed_ / max_acc_;
                acc_distance_ = 0.5 * max_acc_ * acc_time_ * acc_time_;
                max_speed_ = sqrt(max_acc_ * length_); // 重新计算实际最大速度
            }
            
            // 计算匀速段的时间和距离
            cruise_distance_ = length_ - 2 * acc_distance_;
            cruise_time_ = (cruise_distance_ > 1e-6) ? cruise_distance_ / max_speed_ : 0.0;
            
            // 计算单程总时间
            one_way_time_ = 2 * acc_time_ + cruise_time_;
            
            // 调试信息
            std::cout << "Back and forth generator initialized:" << std::endl;
            std::cout << "  Length: " << length_ << " m" << std::endl;
            std::cout << "  Max speed: " << max_speed_ << " m/s" << std::endl;
            std::cout << "  Max acc: " << max_acc_ << " m/s²" << std::endl;
            std::cout << "  Acc time: " << acc_time_ << " s" << std::endl;
            std::cout << "  Acc distance: " << acc_distance_ << " m" << std::endl;
            std::cout << "  Cruise time: " << cruise_time_ << " s" << std::endl;
            std::cout << "  One way time: " << one_way_time_ << " s" << std::endl;
        };

        gtsam::Vector3 pos(double t);

        gtsam::Vector3 vel(double t);

        gtsam::Vector3 theta(double t);

        gtsam::Vector3 omega(double t);

        gtsam::Vector3 thrust(double t);

        gtsam::Vector4 inputfm(double t);

        gtsam::Vector4 input(double t);

    private:
        double length_;          // 直线长度
        double max_speed_;       // 最大速度
        double dt_;              // 时间步长
        double max_acc_;         // 最大加速度
        double x_offset_ = -2., y_offset_ = 0.10, z_offset_ = 0.;        // X axis offset

        // 计算得到的运动参数
        double acc_time_;        // 加速/减速段时间
        double acc_distance_;    // 加速/减速段距离
        double cruise_time_;     // 匀速段时间
        double cruise_distance_; // 匀速段距离
        double one_way_time_;    // 单程总时间

        gtsam::Vector3 g_ = gtsam::Vector3(0, 0, 9.81);
        DynamicsParams dynamics_params_;
        
        // 辅助函数：计算在单程内的运动状态
        void get_segment_state(double t, int& direction, double& normalized_time, 
                            double& s, double& v, double& a);
    };

    class circle_generator
    {
    public:
        circle_generator(double radius, double speed, double dt)
            : radius_(radius),
              speed_(speed),
              dt_(dt){};

        gtsam::Vector3 pos(double t);

        gtsam::Vector3 vel(double t);

        gtsam::Vector3 theta(double t);

        gtsam::Vector3 omega(double t);

        gtsam::Vector3 thrust(double t);

        gtsam::Vector4 inputfm(double t);

        gtsam::Vector4 input(double t);

    private:
        double radius_;
        double speed_;
        double dt_;

        gtsam::Vector3 g_ = gtsam::Vector3(0, 0, 9.81);
        DynamicsParams dynamics_params_;
    };

    class vertical_circle_generator
    {
    public:
        vertical_circle_generator(double radius, double speed, double dt)
            : radius_(radius),
              speed_(speed),
              dt_(dt){};

        gtsam::Vector3 pos(double t);

        gtsam::Vector3 vel(double t);

        gtsam::Vector3 theta(double t);

        gtsam::Vector3 omega(double t);

        gtsam::Vector3 thrust(double t);

        gtsam::Vector4 inputfm(double t);

        gtsam::Vector4 input(double t);

    private:
        double radius_;
        double speed_;
        double dt_;

        gtsam::Vector3 g_ = gtsam::Vector3(0, 0, 9.81);
        DynamicsParams dynamics_params_;
    };


    class cir_conacc_generator
    {
    public:
        cir_conacc_generator(double radius, double max_speed, double acc, double dt)
            : radius_(radius),
              max_speed_(max_speed),
              acc_(acc),
              dt_(dt){};

        double angle(double t);
        
        gtsam::Vector3 pos(double t);

        gtsam::Vector3 vel(double t);

        gtsam::Vector3 theta(double t);

        gtsam::Vector3 omega(double t);

        gtsam::Vector3 thrust(double t);

        gtsam::Vector4 inputfm(double t);

        gtsam::Vector4 input(double t);

    private:
        double radius_;
        double max_speed_;
        double acc_;
        double dt_;

        gtsam::Vector3 g_ = gtsam::Vector3(0, 0, 9.81);
        DynamicsParams dynamics_params_;
    };

    class figure_eight_generator
    {
    public:
        figure_eight_generator(double scale, double speed, double dt)
            : scale_(scale),
            speed_(speed),
            dt_(dt) {};

        gtsam::Vector3 pos(double t);

        gtsam::Vector3 vel(double t);

        gtsam::Vector3 theta(double t);

        gtsam::Vector3 omega(double t);

        gtsam::Vector3 thrust(double t);

        gtsam::Vector4 inputfm(double t);

        gtsam::Vector4 input(double t);

    private:
        double scale_;  // Controls the size of the figure-eight
        double speed_;  // Controls the speed of traversal
        double dt_;     // Time step for numerical derivatives

        gtsam::Vector3 g_ = gtsam::Vector3(0, 0, 9.81);  // Gravity vector
        DynamicsParams dynamics_params_;                 // Quadrotor dynamics parameters
    };
}

#endif