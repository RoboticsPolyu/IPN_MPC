#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <ipn_mpc/trajectory/trajectory_generator.h>
#include <random>

using namespace gtsam;
using namespace QuadrotorSim_SO3;
using namespace std;
using namespace Trajectory;
using namespace UAVFactor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(int argc, char** argv) {
    const std::string simulator_config_path =
        argc > 1 ? argv[1] : "../config/quadrotor_thrust_gyro.yaml";
    IPN_LOG_INFO << "simulator_config=" << simulator_config_path;
    constexpr double kIntegrationStepSeconds = 0.001;
    constexpr double kTrajectoryRadiusMeters = 1.0;
    constexpr double kLinearVelocityMetersPerSecond = 3.0;
    constexpr double kAccelerationMetersPerSecondSquared = 0.01;
    constexpr int kSimulationSteps = 30'000;
    constexpr int kMinimumRotorSpeed = 18'500;
    constexpr int kMaximumRotorSpeed = 18'699;

    double integration_step = kIntegrationStepSeconds;
    cir_conacc_generator circle_generator(kTrajectoryRadiusMeters, kLinearVelocityMetersPerSecond,
                                          kAccelerationMetersPerSecondSquared, integration_step);

    auto dynamics_noise = noiseModel::Diagonal::Sigmas(
        (Vector(12) << Vector3::Constant(0.05), Vector3::Constant(0.05), Vector3::Constant(0.01),
         Vector3::Constant(0.01))
            .finished());

    DynamicsFactorTm dynamics_factor(X(0), V(0), S(0), U(0), X(1), V(1), S(1), 0.1f,
                                     dynamics_noise);

    Quadrotor quad(simulator_config_path);
    State state_0;
    State state_1;

    integration_step = 0.01;
    constexpr double kInitialTimeSeconds = 1.0;

    quad.setState(state_0);

    std::mt19937 random_generator{std::random_device{}()};
    std::uniform_int_distribution<int> rotor_speed_distribution(kMinimumRotorSpeed,
                                                                kMaximumRotorSpeed);

    for (int step = 0; step < kSimulationSteps; ++step) {
        gtsam::Vector4 rotor_velocities;
        rotor_velocities << rotor_speed_distribution(random_generator),
            rotor_speed_distribution(random_generator), rotor_speed_distribution(random_generator),
            rotor_speed_distribution(random_generator);

        const gtsam::Vector4 input = quad.inverseRotorVelocities(rotor_velocities);

        IPN_LOG_DEBUG << "Control wrench [thrust_N, torque_Nm]: " << input.transpose();
        quad.stepODE(integration_step, input);

        if (step == 9) {
            state_1 = quad.getState();
        }

        const State current_state = quad.getState();
        const double sample_time = kInitialTimeSeconds + integration_step * step;
        IPN_LOG_DEBUG << "Simulated state: step=" << step
                      << ", position_m=" << current_state.p.transpose()
                      << ", rotation_logmap_rad=" << Rot3::Logmap(current_state.rot).transpose()
                      << ", velocity_mps=" << current_state.v.transpose()
                      << ", body_rate_radps=" << current_state.body_rate.transpose();
        IPN_LOG_DEBUG << "Reference state: step=" << step
                      << ", position_m=" << circle_generator.pos(sample_time).transpose()
                      << ", velocity_mps=" << circle_generator.vel(sample_time).transpose()
                      << ", attitude_rad=" << circle_generator.theta(sample_time).transpose()
                      << ", angular_velocity_radps="
                      << circle_generator.omega(sample_time).transpose();

        if (!quad.renderHistoryTrj()) {
            break;
        }
    }

    const gtsam::Vector4 input = circle_generator.inputfm(kInitialTimeSeconds);

    Pose3 pose_i(state_0.rot, state_0.p), pose_j(state_1.rot, state_1.p);
    Vector3 vel_i(state_0.v), vel_j(state_1.v), omega_i(state_0.body_rate),
        omega_j(state_1.body_rate);

    IPN_LOG_DEBUG << "Dynamics factor initial pose: " << pose_i;
    IPN_LOG_DEBUG << "Dynamics factor final pose: " << pose_j;

    Matrix H_e_posei, H_e_posej;
    Matrix H_e_vi, H_e_oi, H_e_vj, H_e_oj;
    Matrix H_e_ui;

    Vector12 err = dynamics_factor.evaluateError(pose_i, vel_i, omega_i, input, pose_j, vel_j,
                                                 omega_j, &H_e_posei, &H_e_vi, &H_e_oi, &H_e_ui,
                                                 &H_e_posej, &H_e_vj, &H_e_oj);
    IPN_LOG_DEBUG << "Dynamics factor residual: value=" << err.transpose();

    while (quad.renderHistoryTrj()) {
    }

    return 0;
}
