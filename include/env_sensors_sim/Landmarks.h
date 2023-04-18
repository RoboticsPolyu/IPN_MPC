#pragma once

#include <Eigen/Core>
#include <iostream>

#include <vector>

#include "Common.h"

namespace ENV_SIM
{
    class Landmarks
    {
    public:
        Landmarks();

    private:
        Features features_;
    };
}