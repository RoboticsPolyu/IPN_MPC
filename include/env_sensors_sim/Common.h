#pragma once

#include <Eigen/Core>
#include <iostream>

#include "gtsam_wrapper.h"

// Point feature
struct Feature
{
    uint32_t frame_id;
    uint32_t feature_id;
    double x;
    double y;
    double z;
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