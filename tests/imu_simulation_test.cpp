#include <cmath>
#include <iostream>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <ipn_mpc/simulation/imu.h>
#include <stdexcept>

namespace {
constexpr double kTolerance = 1.0e-10;

bool near(const gtsam::Vector3& actual, const gtsam::Vector3& expected, double tolerance,
          const char* name) {
    if ((actual - expected).norm() <= tolerance) return true;
    std::cerr << name << " mismatch: " << actual.transpose() << " vs " << expected.transpose()
              << '\n';
    return false;
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected quadrotor configuration path\n";
        return 1;
    }

    Sensors_Sim::IMU ideal_imu(0.0, 0.0, 0.0, 0.0, 9.81, 7);
    State state;
    state.rot = gtsam::Rot3::RzRyRx(0.3, -0.2, 0.1);
    state.body_rate = gtsam::Vector3(0.4, -0.5, 0.6);
    const gtsam::Vector3 expected_specific_force(1.0, 2.0, 3.0);
    const gtsam::Vector3 world_acceleration =
        gtsam::Vector3(0.0, 0.0, -9.81) + state.rot.rotate(expected_specific_force);

    const IMUMeasurement first = ideal_imu.measure(state, world_acceleration, 0.01);
    if (first.idx != 0 || !near(first.true_acc, expected_specific_force, kTolerance, "specific force") ||
        !near(first.acc, expected_specific_force, kTolerance, "ideal acceleration") ||
        !near(first.angular_speed, state.body_rate, kTolerance, "ideal angular speed"))
        return 2;

    const gtsam::Vector3 accel_bias(0.1, -0.2, 0.3);
    const gtsam::Vector3 gyro_bias(-0.01, 0.02, -0.03);
    ideal_imu.reset(accel_bias, gyro_bias);
    const IMUMeasurement biased = ideal_imu.measure(state, world_acceleration, 0.01);
    if (biased.idx != 0 || !near(biased.acc, expected_specific_force + accel_bias, kTolerance,
                                 "biased acceleration") ||
        !near(biased.angular_speed, state.body_rate + gyro_bias, kTolerance,
              "biased angular speed"))
        return 3;

    bool rejected_bad_dt = false;
    try {
        ideal_imu.measure(state, world_acceleration, 0.0);
    } catch (const std::invalid_argument&) {
        rejected_bad_dt = true;
    }
    if (!rejected_bad_dt) {
        std::cerr << "IMU accepted a zero sample interval\n";
        return 4;
    }

    QuadrotorSim_SO3::Quadrotor quadrotor(argv[1], false);
    const double hover_rpm =
        std::sqrt(quadrotor.getMass() * quadrotor.getGravity() /
                  (4.0 * quadrotor.getPropellerThrustCoefficient()));
    quadrotor.setInput(hover_rpm, hover_rpm, hover_rpm, hover_rpm);
    quadrotor.step(0.01);
    const IMUMeasurement& integrated = quadrotor.getIMUMeasurement();
    if (integrated.idx != 0 || std::abs(integrated.timestamp - 0.01) > kTolerance ||
        !integrated.acc.allFinite() ||
        !integrated.angular_speed.allFinite() ||
        std::abs(integrated.true_acc.z() - quadrotor.getGravity()) > 1.0e-8) {
        std::cerr << "Quadrotor did not publish a valid hover IMU sample\n";
        return 5;
    }

    std::cout << "IMU model and quadrotor integration tests passed\n";
    return 0;
}
