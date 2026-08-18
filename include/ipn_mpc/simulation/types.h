#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <ipn_mpc/factors/gtsam_compatibility.h>
#include <string>
#include <vector>

enum PointType { L_NONV = 0, L_VIS = 1 };

// Point feature
struct Feature {
    uint32_t frame_id{0};
    uint32_t feature_id{0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    uint8_t type{L_NONV};
};
using Features = std::vector<Feature>;

struct IMUMeasurement {
    int32_t idx{0};
    double timestamp{0.0};
    gtsam::Vector3 acc{gtsam::Vector3::Zero()};
    gtsam::Vector3 angular_speed{gtsam::Vector3::Zero()};

    gtsam::Vector3 true_acc{gtsam::Vector3::Zero()};
    gtsam::Vector3 true_angular_speed{gtsam::Vector3::Zero()};

    gtsam::Vector3 acc_bias{gtsam::Vector3::Zero()};
    gtsam::Vector3 angular_speed_bias{gtsam::Vector3::Zero()};
};

enum class MotionType { Stationary = 0, Linear = 1, Ellipse = 2, PingPong = 3 };

struct Style {
    MotionType motion_type{MotionType::Stationary};
};

struct State {
    int64_t id{0};
    double timestamp{0.0};
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v{Eigen::Vector3d::Zero()};
    gtsam::Rot3 rot{gtsam::Rot3::Identity()};
    Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
    Eigen::Vector4d thrust_torque{Eigen::Vector4d::Zero()};
    Eigen::Array4d motor_rpm{Eigen::Array4d::Zero()};
};

// Define a structure to hold 3D points
struct Point3D {
    float timestamp{0.0F};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

enum ObsType { sphere = 0, box = 1, cylinder = 2 };

// Define a structure of obstacles
struct Obstacle {
    float timestamp{0.0F};
    ObsType obs_type{sphere};
    gtsam::Vector3 obs_vel{gtsam::Vector3::Zero()};
    gtsam::Vector3 obs_pos{gtsam::Vector3::Zero()};
    float obs_size{0.0F};
    float obs_height{0.0F};
    gtsam::Vector3 half_extents{gtsam::Vector3::Zero()};
    MotionType motion_type{MotionType::Stationary};
    gtsam::Vector3 initial_pos{gtsam::Vector3::Zero()};
    gtsam::Vector3 motion_axis{gtsam::Vector3::UnitX()};
    gtsam::Vector3 motion_amplitude{gtsam::Vector3::Zero()};
    double motion_speed{0.0};
    double motion_phase{0.0};
    std::string name;
};
