#include <ipn_mpc/control/energy_control_factor.h>
#include <cmath>
#include <stdexcept>

EnergyFactor::EnergyFactor(Key p_i, Key vel_i, Key input_i, Key dt_i, gtsam::Vector3 drag_k,
                           gtsam::Vector3 model_params, const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, input_i, dt_i), drag_k_(std::move(drag_k)),
      model_params_(std::move(model_params)) {
    if (!model || model->dim() != 1)
        throw std::invalid_argument("EnergyFactor requires a 1D noise model");
    if (!drag_k_.allFinite() || !model_params_.allFinite())
        throw std::invalid_argument("EnergyFactor parameters must be finite");
}

Vector EnergyFactor::evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                                   const gtsam::Vector4& control_input, const float& dt,
                                   gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
                                   gtsam::OptionalMatrixType H3,
                                   gtsam::OptionalMatrixType H4) const {
    return evaluateError(pose, velocity, control_input, static_cast<double>(dt), H1, H2, H3, H4);
}

Vector EnergyFactor::evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                                   const gtsam::Vector4& control_input, double dt,
                                   gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
                                   gtsam::OptionalMatrixType H3,
                                   gtsam::OptionalMatrixType H4) const {
    gtsam::Matrix3 body_velocity_rotation;
    gtsam::Matrix3 body_velocity_velocity;
    const gtsam::Vector3 body_velocity =
        pose.rotation().unrotate(velocity, body_velocity_rotation, body_velocity_velocity);

    const gtsam::Vector3 drag_force_body = drag_k_.cwiseProduct(body_velocity);
    const double drag_power = -body_velocity.dot(drag_force_body);

    const double c0 = model_params_(0);
    const double c1 = model_params_(1);
    const double c2 = model_params_(2);
    const double actuator_power =
        (gtsam::Vector4::Constant(c0).array() + c1 * control_input.array() +
         c2 * control_input.array().square()).sum();
    const double total_power = actuator_power + drag_power;

    const gtsam::Matrix13 drag_power_body_velocity =
        (-2.0 * drag_force_body).transpose();
    if (H1) {
        H1->setZero(1, 6);
        H1->block<1, 3>(0, 0) =
            dt * drag_power_body_velocity * body_velocity_rotation;
    }
    if (H2)
        *H2 = dt * drag_power_body_velocity * body_velocity_velocity;
    if (H3)
        *H3 = dt * (gtsam::Vector4::Constant(c1) + 2.0 * c2 * control_input).transpose();
    if (H4) {
        H4->resize(1, 1);
        (*H4)(0, 0) = total_power;
    }

    return gtsam::Vector1(dt * total_power);
}
