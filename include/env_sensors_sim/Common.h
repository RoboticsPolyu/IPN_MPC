#pragma once

#include <Eigen/Core>
#include <iostream>

#include "gtsam_wrapper.h"

enum PointType
{
    L_NONV = 0,
    L_VIS  = 1
};

// Point feature
struct Feature
{
    uint32_t frame_id;
    uint32_t feature_id;
    double x;
    double y;
    double z;
    uint8_t type; // 0 nonvisible 1 visible
};
typedef std::vector<Feature> Features;

struct IMUMeasurement
{
    int32_t idx;
    gtsam::Vector3 acc;
    gtsam::Vector3 angular_speed;

    gtsam::Vector3 true_acc;
    gtsam::Vector3 true_angular_speed;

    gtsam::Vector3 acc_bias;
    gtsam::Vector3 angular_speed_bias;
};

enum Motion_type
{
    Stationary = 1,
    Linear = 2 
};

typedef struct Style
{
    Motion_type motion_type;

}Style;

typedef struct State
{
    int64_t         id;
    double          timestamp;

    Eigen::Vector3d p;
    Eigen::Vector3d v;
    gtsam::Rot3     rot;
    Eigen::Vector3d body_rate;

    double          acc_z;
    Eigen::Vector3d torque;

    Eigen::Vector4d thrust_torque;
    Eigen::Array4d  motor_rpm;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // State(): id(0), timestamp(0), p(Eigen::Vector3d::Zero()), v(Eigen::Vector3d::Zero()),
    //     rot(gtsam::Rot3::identity()), body_rate(Eigen::Vector3d::Zero()), acc_z(0), torque(Eigen::Vector3d::Zero()), 
    //     thrust_torque(Eigen::Vector4d::Zero()), motor_rpm(Eigen::Array4d::Zero()) {};

    // const State operator=(const State& state)
    // {
    //     id = state.id;
    //     timestamp = state.timestamp;
    //     p = state.p;
    //     v = state.v;
    //     rot = state.rot;
    //     body_rate = state.body_rate;
    //     acc_z = state.acc_z;
    //     torque = state.torque;
    //     thrust_torque = state.thrust_torque;
    //     motor_rpm = state.motor_rpm;
    //     return *this;
    // }

    // state = state will dead ???

}State;

// Define a structure to hold 3D points
typedef struct Point3D 
{
    float timestamp;
    float x, y, z;
}Point3D;


enum ObsType
{
    sphere = 0,
    box    = 1
};


// Define a structure of obstacles
typedef struct Obstacle 
{
    float timestamp;
    ObsType obs_type;
    gtsam::Vector3 obs_vel;
    gtsam::Vector3 obs_pos;

}Obstacle;