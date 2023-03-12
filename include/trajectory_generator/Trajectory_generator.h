#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include <Eigen/Core>
#include "gtsam_wrapper.h"

namespace Trajectory
{
    class Trajectory_generator
    {
    public:
        Trajectory_generator();

    private:
    };

    template<class T>
    class Path
    {
        public:
        Path();
        
        T generate(double time);

        private:
        
    };
}

#endif