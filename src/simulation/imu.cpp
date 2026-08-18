#include <ipn_mpc/simulation/imu.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace Sensors_Sim {

IMU::IMU(double accel_noise_sigma, double gyro_noise_sigma, double accel_bias_rw_sigma,
         double gyro_bias_rw_sigma, double gravity, std::uint32_t seed)
    : accel_noise_sigma_(accel_noise_sigma), gyro_noise_sigma_(gyro_noise_sigma),
      accel_bias_rw_sigma_(accel_bias_rw_sigma), gyro_bias_rw_sigma_(gyro_bias_rw_sigma),
      gravity_(gravity), generator_(seed) {
    validateSigma(accel_noise_sigma_, "accelerometer noise");
    validateSigma(gyro_noise_sigma_, "gyroscope noise");
    validateSigma(accel_bias_rw_sigma_, "accelerometer bias random walk");
    validateSigma(gyro_bias_rw_sigma_, "gyroscope bias random walk");
    setGravity(gravity_);
}

void IMU::validateSigma(double sigma, const char* name) {
    if (!std::isfinite(sigma) || sigma < 0.0)
        throw std::invalid_argument(std::string("IMU ") + name + " sigma must be finite and >= 0");
}

void IMU::setGravity(double gravity) {
    if (!std::isfinite(gravity) || gravity < 0.0)
        throw std::invalid_argument("IMU gravity must be finite and >= 0");
    gravity_ = gravity;
}

void IMU::reset(const gtsam::Vector3& accel_bias, const gtsam::Vector3& gyro_bias) {
    if (!accel_bias.allFinite() || !gyro_bias.allFinite())
        throw std::invalid_argument("IMU biases must be finite");
    accel_bias_ = accel_bias;
    gyro_bias_ = gyro_bias;
    next_index_ = 0;
}

gtsam::Vector3 IMU::sampleNoise(double sigma) {
    return sigma * gtsam::Vector3(standard_normal_(generator_), standard_normal_(generator_),
                                  standard_normal_(generator_));
}

IMUMeasurement IMU::measure(const State& state, const gtsam::Vector3& world_acceleration,
                            double dt) {
    if (!std::isfinite(dt) || dt <= 0.0)
        throw std::invalid_argument("IMU sample interval must be finite and > 0");
    if (!world_acceleration.allFinite() || !state.body_rate.allFinite())
        throw std::invalid_argument("IMU truth inputs must be finite");

    const double sqrt_dt = std::sqrt(dt);
    accel_bias_ += sampleNoise(accel_bias_rw_sigma_ * sqrt_dt);
    gyro_bias_ += sampleNoise(gyro_bias_rw_sigma_ * sqrt_dt);

    const gtsam::Vector3 gravity_world(0.0, 0.0, -gravity_);
    IMUMeasurement measurement;
    measurement.idx = next_index_++;
    measurement.timestamp = state.timestamp;
    measurement.true_acc = state.rot.unrotate(world_acceleration - gravity_world);
    measurement.true_angular_speed = state.body_rate;
    measurement.acc_bias = accel_bias_;
    measurement.angular_speed_bias = gyro_bias_;
    measurement.acc =
        measurement.true_acc + accel_bias_ + sampleNoise(accel_noise_sigma_ / sqrt_dt);
    measurement.angular_speed = measurement.true_angular_speed + gyro_bias_ +
                                sampleNoise(gyro_noise_sigma_ / sqrt_dt);
    return measurement;
}

} // namespace Sensors_Sim
