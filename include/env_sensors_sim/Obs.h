#pragma once

#include <Eigen/Core>
#include <iostream>

#include "Common.h"

namespace Obs_Sim
{
    class PointObs
    {
    public:
        PointObs(gtsam::Vector3 obs_center, float radius)
            : obs_center_(obs_center), radius_(radius)
        {
        }
        void SetStyle(const Style& style, const gtsam::Vector3& vel);
        
        const std::vector<gtsam::Vector3> & Outline()
        {
            time++;
        }

        float solveDistance();

    private:
        gtsam::Vector3 obs_center_;
        float radius_;
        uint64_t time = 0;
    };

}