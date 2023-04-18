#pragma once

#include <Eigen/Core>
#include <iostream>
#include <map>

#include "Common.h"
#include "gtsam_wrapper.h"

namespace Sensors_Sim
{
    class IMU
    {
    private:
        double accel_noise_sigma_ = 0.0003924;
        double gyro_noise_sigma_ = 0.000205689024915;
        double accel_bias_rw_sigma_ = 0.004905;
        double gyro_bias_rw_sigma_ = 0.000001454441043;

    public:
        IMU(double accel_noise_sigma, double gyro_noise_sigma, double accel_bias_rw_sigma, double gyro_bias_rw_sigma);

        template <class T>
        IMUMeasurement Measurement(T &trajectory, float timestamp);

        ~IMU();
    };

}