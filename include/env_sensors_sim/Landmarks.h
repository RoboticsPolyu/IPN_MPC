#pragma once

#include <Eigen/Core>
#include <iostream>

#include <vector>

#include "Common.h"

namespace Env_Sim
{
    class Landmarks
    {
    public:
        Landmarks(float map_x, float map_y, float map_center_x, float map_center_y, uint32_t feature_size);

        Features GetMeasurements()
        {
            return features_;
        };

    private:
        Features features_;

        float map_x_;
        float map_y_;
        float map_center_x_;
        float map_center_y_;
        float feature_size_;
    };
}