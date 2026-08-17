#pragma once

#include <array>
#include <ipn_mpc/control/dynamics_params.h>
#include <ipn_mpc/factors/gtsam_compatibility.h>

using namespace gtsam_wrapper;

namespace UAVFactor {
inline constexpr int kMaxActuators = 16;
inline constexpr int kEffectivenessAxes = 6;

struct Rotor {
    double thrust_coefficient{0.0};
    double torque_ratio{0.0};
    gtsam::Vector3 axis{gtsam::Vector3::Zero()};
    gtsam::Rot3 rotation{gtsam::Rot3::Identity()};
    gtsam::Vector3 position{gtsam::Vector3::Zero()};
};

struct Geometry {
    uint8_t num_rotors = 0u;
    std::array<Rotor, kMaxActuators> rotors{};

    bool propeller_torque_disabled = false;
    bool yaw_by_differential_thrust_disabled = false;
    bool propeller_torque_disabled_non_upwards = false;
    bool three_dimensional_thrust_disabled = false;
};

using ActuatorEnabled = Eigen::Matrix<bool, kMaxActuators, 1>;
using EffectivenessMatrix = Eigen::Matrix<float, kEffectivenessAxes, kMaxActuators>;

class ActuatorEffectivenessRotors {
  public:
    ActuatorEffectivenessRotors() = default;
    virtual ~ActuatorEffectivenessRotors() = default;

    ActuatorEffectivenessRotors(const ActuatorEnabled& actuator_enabled, const Geometry& geometry)
        : geometry_(geometry), actuator_enabled_(actuator_enabled) {}

    EffectivenessMatrix effectivenessMatrix() const;

    int computeEffectivenessMatrix(const Geometry& geometry, EffectivenessMatrix& effectiveness,
                                   const ActuatorEnabled& actuator_enabled) const;

  private:
    Geometry geometry_{};
    ActuatorEnabled actuator_enabled_{ActuatorEnabled::Constant(true)};
};

// Allocation Control Factor <thrust, torques, actuators_output>
class GTSAM_EXPORT AllocationFactor
    : public NoiseModelFactor3<double, gtsam::Vector3, gtsam::Vector4> {
  public:
    using shared_ptr = boost::shared_ptr<AllocationFactor>;

    AllocationFactor() = default;

    AllocationFactor(Key thrust_key, Key torques_key, Key actuators_key,
                     const ActuatorEffectivenessRotors& actuator_effectiveness,
                     const SharedNoiseModel& model)
        : Base(model, thrust_key, torques_key, actuators_key),
          actuator_effectiveness_(actuator_effectiveness) {}

    ~AllocationFactor() override = default;

    Vector evaluateError(const double& thrust, const gtsam::Vector3& torques,
                         const gtsam::Vector4& actuator_outputs,
                         gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone) const;

  private:
    using This = AllocationFactor;
    using Base = NoiseModelFactor3<double, gtsam::Vector3, gtsam::Vector4>;

    ActuatorEffectivenessRotors actuator_effectiveness_;
};

} // namespace UAVFactor
