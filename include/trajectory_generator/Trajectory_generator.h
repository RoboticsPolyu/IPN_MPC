#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <Eigen/Core>
#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_factor.h"

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

        }


    private:
        gtsam::Vector3 acc_;
        float yaw_;

        gtsam::Vector3 g_;

    };
}

#endif