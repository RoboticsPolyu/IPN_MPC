#include <algorithm>
#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/dynamics_factor.h>
#include <limits>

namespace UAVFactor {
EffectivenessMatrix ActuatorEffectivenessRotors::effectivenessMatrix() const {
    EffectivenessMatrix effectiveness = EffectivenessMatrix::Zero();
    computeEffectivenessMatrix(geometry_, effectiveness, actuator_enabled_);
    return effectiveness;
}

int ActuatorEffectivenessRotors::computeEffectivenessMatrix(
    const Geometry& geometry, EffectivenessMatrix& effectiveness,
    const ActuatorEnabled& actuator_enabled) const {
    effectiveness.setZero();
    int active_actuator_count = 0;

    const int rotor_count = std::min<int>(geometry.num_rotors, kMaxActuators);
    for (int rotor_index = 0; rotor_index < rotor_count; ++rotor_index) {
        if (!actuator_enabled(rotor_index)) {
            continue;
        }

        gtsam::Vector3 axis = geometry.rotors[rotor_index].axis;
        const double axis_norm = axis.norm();

        if (axis_norm > std::numeric_limits<double>::epsilon()) {
            axis /= axis_norm;
        } else {
            continue;
        }

        const gtsam::Vector3& position = geometry.rotors[rotor_index].position;
        const double thrust_coefficient = geometry.rotors[rotor_index].thrust_coefficient;
        double torque_ratio = geometry.rotors[rotor_index].torque_ratio;

        if (geometry.propeller_torque_disabled) {
            torque_ratio = 0.0;
        }

        if (geometry.propeller_torque_disabled_non_upwards) {
            const bool points_upward =
                std::abs(axis.x()) < 0.1 && std::abs(axis.y()) < 0.1 && axis.z() < -0.5;

            if (!points_upward) {
                torque_ratio = 0.0;
            }
        }

        if (std::abs(thrust_coefficient) < std::numeric_limits<double>::epsilon()) {
            continue;
        }

        const gtsam::Vector3 thrust = thrust_coefficient * axis;
        const gtsam::Vector3 torque =
            thrust_coefficient * (position.cross(axis) - torque_ratio * axis);

        effectiveness.block<3, 1>(0, rotor_index) = torque.cast<float>();
        effectiveness.block<3, 1>(3, rotor_index) = thrust.cast<float>();

        if (geometry.yaw_by_differential_thrust_disabled) {
            // set yaw effectiveness to 0 if yaw is controlled by other means (e.g. tilts)
            effectiveness(2, rotor_index) = 0.0F;
        }

        if (geometry.three_dimensional_thrust_disabled) {
            // Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have
            // a x-component in FW, and z in MC), pass the vector magnitude as z-component, plus the
            // collective tilt. Passing 3D thrust plus tilt is not feasible as they can't be
            // allocated independently, and with the current controller it's not possible to have
            // collective tilt calculated by the allocator directly.

            effectiveness(3, rotor_index) = 0.0F;
            effectiveness(4, rotor_index) = 0.0F;
            effectiveness(5, rotor_index) = static_cast<float>(-thrust_coefficient);
        }

        ++active_actuator_count;
    }

    return active_actuator_count;
}

Vector AllocationFactor::evaluateError(const double& /*thrust*/, const gtsam::Vector3& /*torques*/,
                                       const gtsam::Vector4& /*actuator_outputs*/,
                                       gtsam::OptionalMatrixType /*H1*/,
                                       gtsam::OptionalMatrixType /*H2*/,
                                       gtsam::OptionalMatrixType /*H3*/) const {
    // This legacy factor is retained for API compatibility but has no implemented model yet.
    return gtsam::Vector4::Zero();
}
} // namespace UAVFactor
