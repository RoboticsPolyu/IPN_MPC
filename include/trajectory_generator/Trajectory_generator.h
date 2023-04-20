#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <Eigen/Core>
#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_params.h"

using namespace UAVFactor;

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

}

#endif