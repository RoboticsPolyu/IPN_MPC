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
        Lidar(float range)
            : range_(range)
        {
        }

        
        Features Measurement(T &env, gtsam::Pose3 &body_pose);

    private:
        Features features_;
        float range_;
    };

}