#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <Eigen/Core>
#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_factor.h"

using namespace uav_factor;

namespace Trajectory
{
    class Trajectory_generator
    {
    public:
        Trajectory_generator();

    private:
    };

    template<class T>
    class Path
    {
        public:
        Path();
        
        T generate(double time);

        private:
        
    };

    class simple_trajectory_generator
    {
    public:
        simple_trajectory_generator(gtsam::Vector3 acc, float yaw)
            : acc_(acc), yaw_(yaw){
                g_ = gtsam::Vector3(0, 0, -9.81);
            };

        gtsam::Vector3 pos(float t)
        {
            return (0.5 * t * t * acc_);
        }

        gtsam::Vector3 vel(float t)
        {
            return (t * acc_);
        }

        gtsam::Vector3 theta(float t)
        {
            gtsam::Vector3 force;
            force = thrust(t);
            gtsam::Vector3 zB = force / force.norm();
            gtsam::Vector3 xC(cos(yaw_), sin(yaw_), 0);
            gtsam::Vector3 yB = zB.cross(xC)/ zB.cross(xC).norm();
            gtsam::Vector3 xB = yB.cross(zB);

            gtsam::Matrix3 R;
            R.col(0) = xB;
            R.col(1) = yB;
            R.col(2) = zB;
            gtsam::Rot3 rot3(R);
            return gtsam::Rot3::Logmap(rot3);
        }

        
        gtsam::Vector3 mb(float t)
        {
            gtsam::Vector3 alpha(0,0,0);
            gtsam::Vector3 mb(0,0,0);
            return mb;
        }

        gtsam::Vector3 omega(float t)
        {
            return gtsam::Vector3(0,0,0);
        }

        gtsam::Vector3 thrust(float t)
        {
            gtsam::Vector force;
            force = acc_ - g_;
            return force;
        }

        gtsam::Vector4 input(float t)
        {
            gtsam::Vector3 force;
            force = thrust(t);
            std::cout << "force: " << force << std::endl;

            gtsam::Vector4 Tmb(force.norm()* dynamics_params_.mass, 0, 0, 0);
            gtsam::Matrix4 K1;
            K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
                0, 0, dynamics_params_.arm_length * dynamics_params_.k_f, -dynamics_params_.arm_length * dynamics_params_.k_f,
                -dynamics_params_.arm_length * dynamics_params_.k_f, dynamics_params_.arm_length * dynamics_params_.k_f, 0, 0,
                dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m, -dynamics_params_.k_m;
            gtsam::Vector4 input;
            std::cout << "K_: " << K1 << std::endl;

            input = K1.inverse() * Tmb;
            
            input[0] = sqrt(input[0]);
            input[1] = sqrt(input[1]);
            input[2] = sqrt(input[2]);
            input[3] = sqrt(input[3]);

            return input;
        }


    private:
        gtsam::Vector3 acc_;
        float yaw_;

        gtsam::Vector3 g_;
        DynamicsParams dynamics_params_;
    }; // Not tested

    class circle_generator
    {
    public:
        circle_generator(double radius, double speed, double dt)
            : radius_(radius), 
            speed_(speed),
            dt_(dt)
            {
            };

        gtsam::Vector3 pos(double t)
        {
            double round = 2* M_PI* radius_;
            double angular_speed = speed_ / radius_; // 2*pi/(round / speed) = 2*pi/(2*pi*r)*speed = 
            
            return gtsam::Vector3(sin(angular_speed* t)*radius_, cos(angular_speed* t)* radius_, 1.0);
        }

        gtsam::Vector3 vel(double t)
        {
            double round = 2* M_PI* radius_;
            double angular_speed = speed_ / radius_;
            double v_x = (sin(angular_speed* (t + dt_))* radius_ - sin(angular_speed* (t - dt_))* radius_) / (2*dt_);
            double v_y = (cos(angular_speed* (t + dt_))* radius_ - cos(angular_speed* (t - dt_))* radius_) / (2*dt_);
            
            return gtsam::Vector3(v_x, v_y, 0);
            // return gtsam::Vector3(cos(angular_speed* t)* angular_speed* radius_, -sin(angular_speed* t)* angular_speed* radius_, 0);
        }

        gtsam::Vector3 theta(double t)
        {
            gtsam::Vector3 force;
            double round = 2* M_PI* radius_;
            double angular_speed = speed_ / radius_;

            force = thrust(t);
            double yaw_ = - angular_speed* t;
            gtsam::Vector3 zB = force / force.norm();
            gtsam::Vector3 xC(cos(yaw_), sin(yaw_), 0);
            gtsam::Vector3 yB = zB.cross(xC)/ zB.cross(xC).norm();
            gtsam::Vector3 xB = yB.cross(zB);

            gtsam::Matrix3 R;
            R.col(0) = xB;
            R.col(1) = yB;
            R.col(2) = zB;
            gtsam::Rot3 rot3(R);
            return gtsam::Rot3::Logmap(rot3);
        }

        gtsam::Vector3 omega(double t)
        {
            return gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(theta(t - dt_)).between(gtsam::Rot3::Expmap(theta(t + dt_)) ) ) / (2*dt_);
        }

        gtsam::Vector3 thrust(double t)
        {
            gtsam::Vector force;
            // double round = 2* M_PI* radius_;
            double angular_speed = 2* radius_* speed_;
            // force = gtsam::Vector3(-sin(angular_speed* t)* angular_speed* angular_speed* radius_,
            //                 -cos(angular_speed* t)* angular_speed* angular_speed* radius_, 
            //                 0)  + g_;

            force = (vel(t+ dt_) - vel(t - dt_))/(2*dt_)  + g_;
            return force;
        }

        gtsam::Vector4 input(double t)
        {
            gtsam::Vector3 force;
            force = thrust(t); 
            // std::cout << "\n ------force----- \n"  << force;

            gtsam::Vector3 d_omega = (omega(t+ dt_) - omega(t - dt_) )/ (2*dt_);
            gtsam::Matrix33 J_inv;
            J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
            0, 1.0 / dynamics_params_.Iyy, 0,
            0, 0, 1.0 / dynamics_params_.Izz;
            gtsam::Matrix33 J;
            J << dynamics_params_.Ixx, 0, 0,
            0, dynamics_params_.Iyy, 0,
            0, 0, dynamics_params_.Izz; 

            gtsam::Vector3 mb = J* d_omega + omega(t).cross(J* omega(t) );
            gtsam::Vector4 Tmb(force.norm()* dynamics_params_.mass, mb(0), mb(1), mb(2));
            
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


    private:
        double radius_;
        double speed_;
        double dt_;

        gtsam::Vector3 g_ = gtsam::Vector3(0, 0, 9.81);
        DynamicsParams dynamics_params_;
    };


}

#endif