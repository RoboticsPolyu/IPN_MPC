#pragma once

#include <cstdint>
#include <ipn_mpc/factors/gtsam_compatibility.h>
#include <ipn_mpc/simulation/types.h>
#include <random>

namespace Sensors_Sim {

/** Strapdown IMU simulator with white measurement noise and bias random walks.
 *
 * Noise parameters are continuous-time densities. For a sample interval dt,
 * measurement noise is scaled by 1/sqrt(dt), while bias increments are scaled
 * by sqrt(dt). The accelerometer output is body-frame specific force:
 *
 *   f_b = R_wb^T (a_w - g_w).
 */
class IMU {
  public:
    IMU(double accel_noise_sigma = 0.0003924,
        double gyro_noise_sigma = 0.000205689024915,
        double accel_bias_rw_sigma = 0.004905,
        double gyro_bias_rw_sigma = 0.000001454441043, double gravity = 9.81,
        std::uint32_t seed = std::random_device{}());

    IMUMeasurement measure(const State& state, const gtsam::Vector3& world_acceleration,
                           double dt);

    IMUMeasurement Measurement(const State& state,
                               const gtsam::Vector3& world_acceleration, double dt) {
        return measure(state, world_acceleration, dt);
    }

    void reset(const gtsam::Vector3& accel_bias = gtsam::Vector3::Zero(),
               const gtsam::Vector3& gyro_bias = gtsam::Vector3::Zero());
    void setGravity(double gravity);

    const gtsam::Vector3& accelBias() const { return accel_bias_; }
    const gtsam::Vector3& gyroBias() const { return gyro_bias_; }

  private:
    gtsam::Vector3 sampleNoise(double sigma);
    static void validateSigma(double sigma, const char* name);

    double accel_noise_sigma_;
    double gyro_noise_sigma_;
    double accel_bias_rw_sigma_;
    double gyro_bias_rw_sigma_;
    double gravity_;
    gtsam::Vector3 accel_bias_{gtsam::Vector3::Zero()};
    gtsam::Vector3 gyro_bias_{gtsam::Vector3::Zero()};
    std::mt19937 generator_;
    std::normal_distribution<double> standard_normal_{0.0, 1.0};
    std::int32_t next_index_{0};
};

} // namespace Sensors_Sim
