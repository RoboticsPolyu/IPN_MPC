#pragma once

#include <Eigen/Core>
#include <iostream>

#include "Common.h"
#include "Landmarks.h"


namespace SENSORS_SIM
{
    class Lidar
    {
    public:
        Lidar();

        template <class T>
        Features Measurement(T &env);

    private:
        Features features_;
        float range_;
    };

}