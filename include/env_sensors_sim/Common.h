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
    Eigen::Vector4d thrust_torque;
    Eigen::Array4d  motor_rpm;

    State() 
        : p(0,0,0), rot(gtsam::Rot3::identity()), v(0,0,0), body_rate(0,0,0), thrust_torque(0,0,0,0), motor_rpm(0,0,0,0){}
    
    State(const State& other)
        : p(other.p), rot(other.rot), v(other.v), body_rate(other.body_rate), thrust_torque(other.thrust_torque), motor_rpm(other.motor_rpm){}
    
    State& operator=(const State& other) {
        if (this != &other) {
            p = other.p;
            rot = other.rot;
            v = other.v;
            body_rate = other.body_rate;
            thrust_torque = other.thrust_torque;
            motor_rpm = other.motor_rpm;
        }
        return *this;
    }

}State;

// Define a structure to hold 3D points
typedef struct Point3D 
{
    float timestamp;
    float x, y, z;
}Point3D;


enum ObsType
{
    sphere   = 0,
    box      = 1,
    cylinder = 2
};

// Define a structure of obstacles
typedef struct Obstacle 
{
    float timestamp;
    ObsType obs_type;
    gtsam::Vector3 obs_vel;
    gtsam::Vector3 obs_pos;
    float obs_size;

}Obstacle;