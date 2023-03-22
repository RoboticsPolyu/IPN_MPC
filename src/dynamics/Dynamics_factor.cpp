#include "quadrotor_simulator/Dynamics_factor.h"


namespace uav_factor
{
DynamicsFactor::DynamicsFactor(Key p_i, Key vel_i, Key rot_i, Key omega_i, Key input_i,
                               Key p_j, Key vel_j, Key rot_j, Key omega_j,
                               const Dynamics &dys)
    : Base(noiseModel::Gaussian::Covariance(dys.MeasCov_), p_i, vel_i, rot_i, omega_i, input_i,
           p_j, vel_j, rot_j, omega_j),
      _DYS_(dys)
{
};

DynamicsFactor2::DynamicsFactor2(Key x_i, Key input_i, Key x_j, const SharedNoiseModel& model)
    : Base(model, x_i, input_i, x_j)
{
};




}