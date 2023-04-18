#include <env_sensors_sim/IMU.h>

namespace Sensors_Sim
{

    IMU::IMU(double accel_noise_sigma, double gyro_noise_sigma, double accel_bias_rw_sigma, double gyro_bias_rw_sigma) : accel_noise_sigma_(accel_noise_sigma), gyro_noise_sigma_(gyro_noise_sigma),
                                                                                                                         accel_bias_rw_sigma_(accel_bias_rw_sigma), gyro_bias_rw_sigma_(gyro_bias_rw_sigma)
    {
    }

    IMU::~IMU()
    {
    }

}