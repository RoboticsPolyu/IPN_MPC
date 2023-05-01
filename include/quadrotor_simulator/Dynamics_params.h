#ifndef __DYNAMICS_PARAMS_H__
#define __DYNAMICS_PARAMS_H__

#include <Eigen/Core>
#include <iostream>


namespace UAVFactor
{
    struct DynamicsParams
    {   
        float  g            = 9.81;
        float  mass         = 0.98;
        double Ixx          = 2.64e-3;
        double Iyy          = 2.64e-3;
        double Izz          = 4.96e-3;
        double k_f          = 8.98132e-9;
        double prop_radius_ = 0.062;
        double k_m          = 0.07 * (3 * prop_radius_) * k_f;
        float  min_rpm      = 1200;
        float  max_rpm      = 35000;
        float  arm_length   = 0.26;
        double esc_factor   = 0.7;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

}

#endif