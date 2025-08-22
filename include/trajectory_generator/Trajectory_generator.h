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