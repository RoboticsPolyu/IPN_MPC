#pragma once

#include <Eigen/Core>
#include <iostream>

#include "Common.h"
#include "Landmarks.h"

namespace Sensors_Sim
{
    template <typename T>
    class Lidar
    {
    public:
        Lidar(float range, float range_min)
            : range_(range), range_min_(range_min)
        {
        }

        
        Features Measurement(T &env, gtsam::Pose3 &body_pose);

    private:
        Features features_;
        float range_;
        float range_min_;
    };

}