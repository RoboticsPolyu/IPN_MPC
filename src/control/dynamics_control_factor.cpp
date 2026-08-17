#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/dynamics_control_factor.h>
#include <gtsam/base/numericalDerivative.h>
#include <stdexcept>
#include <type_traits>

namespace UAVFactor {
namespace {
const gtsam::Vector3 kBodyZ = gtsam::Vector3::UnitZ();

gtsam::Matrix3 diagonalMatrix(const gtsam::Vector3& diagonal) {
    return diagonal.asDiagonal();
}
} // namespace

TerminalDynamicsFactor::TerminalDynamicsFactor(
    Key pose_0, Key velocity_0, Key body_rate_0, const gtsam::KeyVector& control_keys, Key pose_j,
    Key velocity_j, Key body_rate_j, double dt, const SharedNoiseModel& model,
    const DynamicsParams& dynamics_params)
    : NoiseModelFactor(model, [&] {
          gtsam::KeyVector keys{pose_0, velocity_0, body_rate_0};
          keys.insert(keys.end(), control_keys.begin(), control_keys.end());
          keys.insert(keys.end(), {pose_j, velocity_j, body_rate_j});
          return keys;
      }()),
      pose_0_(pose_0), velocity_0_(velocity_0), body_rate_0_(body_rate_0),
      control_keys_(control_keys), pose_j_(pose_j), velocity_j_(velocity_j),
      body_rate_j_(body_rate_j), dt_(dt), dynamics_params_(dynamics_params) {
    if (!model || model->dim() != 12)
        throw std::invalid_argument("TerminalDynamicsFactor requires a 12D Q_j noise model");
    if (control_keys_.empty())
        throw std::invalid_argument("TerminalDynamicsFactor requires at least one control");
    if (dt_ <= 0.0) throw std::invalid_argument("TerminalDynamicsFactor requires dt > 0");
}

DynamicsState TerminalDynamicsFactor::propagate(const DynamicsState& state,
                                                 const gtsam::Vector4& control) const {
    const gtsam::Vector3 acceleration =
        -dynamics_params_.g * kBodyZ +
        state.pose.rotation().rotate(kBodyZ * control(0)) / dynamics_params_.mass;
    const gtsam::Matrix3 inertia = diagonalMatrix(
        gtsam::Vector3(dynamics_params_.Ixx, dynamics_params_.Iyy, dynamics_params_.Izz));
    const gtsam::Vector3 angular_acceleration =
        inertia.inverse() *
        (control.tail<3>() - state.body_rate.cross(inertia * state.body_rate));

    DynamicsState next;
    next.pose = gtsam::Pose3(
        state.pose.rotation() * gtsam::Rot3::Expmap(state.body_rate * dt_),
        state.pose.translation() + state.velocity * dt_ + 0.5 * acceleration * dt_ * dt_);
    next.velocity = state.velocity + acceleration * dt_;
    next.body_rate = state.body_rate + angular_acceleration * dt_;
    return next;
}

DynamicsState TerminalDynamicsFactor::predict(
    const DynamicsState& initial_state, const std::vector<gtsam::Vector4>& controls) const {
    if (controls.size() != horizon())
        throw std::invalid_argument("Control count must equal TerminalDynamicsFactor horizon");
    DynamicsState predicted = initial_state;
    for (const auto& control : controls) predicted = propagate(predicted, control);
    return predicted;
}

gtsam::Vector TerminalDynamicsFactor::residual(const gtsam::Values& values) const {
    const DynamicsState initial{values.at<gtsam::Pose3>(pose_0_),
                                values.at<gtsam::Vector3>(velocity_0_),
                                values.at<gtsam::Vector3>(body_rate_0_)};
    std::vector<gtsam::Vector4> controls;
    controls.reserve(horizon());
    for (Key key : control_keys_) controls.push_back(values.at<gtsam::Vector4>(key));
    const DynamicsState predicted = predict(initial, controls);
    const gtsam::Pose3& target_pose = values.at<gtsam::Pose3>(pose_j_);

    gtsam::Vector12 error;
    error.segment<3>(0) = target_pose.translation() - predicted.pose.translation();
    error.segment<3>(3) =
        gtsam::Rot3::Logmap(predicted.pose.rotation().between(target_pose.rotation()));
    error.segment<3>(6) = values.at<gtsam::Vector3>(velocity_j_) - predicted.velocity;
    error.segment<3>(9) = values.at<gtsam::Vector3>(body_rate_j_) - predicted.body_rate;
    return error;
}

gtsam::Vector TerminalDynamicsFactor::unwhitenedError(
    const gtsam::Values& values, gtsam::OptionalMatrixVecType jacobians) const {
    const gtsam::Vector error = residual(values);
    if (!jacobians) return error;

    jacobians->resize(keys().size());
    auto derivative = [&](Key key, const auto& value) {
        using Value = std::decay_t<decltype(value)>;
        return gtsam::numericalDerivative11<gtsam::Vector, Value>(
            [&](const Value& perturbed_value) {
                gtsam::Values perturbed(values);
                perturbed.update(key, perturbed_value);
                return residual(perturbed);
            }, value);
    };

    std::size_t index = 0;
    (*jacobians)[index++] = derivative(pose_0_, values.at<gtsam::Pose3>(pose_0_));
    (*jacobians)[index++] = derivative(velocity_0_, values.at<gtsam::Vector3>(velocity_0_));
    (*jacobians)[index++] = derivative(body_rate_0_, values.at<gtsam::Vector3>(body_rate_0_));
    for (Key key : control_keys_)
        (*jacobians)[index++] = derivative(key, values.at<gtsam::Vector4>(key));
    (*jacobians)[index++] = derivative(pose_j_, values.at<gtsam::Pose3>(pose_j_));
    (*jacobians)[index++] = derivative(velocity_j_, values.at<gtsam::Vector3>(velocity_j_));
    (*jacobians)[index] = derivative(body_rate_j_, values.at<gtsam::Vector3>(body_rate_j_));
    return error;
}

TerminalAccelerationGyroFactor::TerminalAccelerationGyroFactor(
    Key pose_0, Key velocity_0, const gtsam::KeyVector& control_keys, Key pose_j, Key velocity_j,
    double dt, const SharedNoiseModel& model)
    : NoiseModelFactor(model, [&] {
          gtsam::KeyVector keys{pose_0, velocity_0};
          keys.insert(keys.end(), control_keys.begin(), control_keys.end());
          keys.insert(keys.end(), {pose_j, velocity_j});
          return keys;
      }()),
      pose_0_(pose_0), velocity_0_(velocity_0), control_keys_(control_keys), pose_j_(pose_j),
      velocity_j_(velocity_j), dt_(dt) {
    if (!model || model->dim() != 9)
        throw std::invalid_argument(
            "TerminalAccelerationGyroFactor requires a 9D Q_j noise model");
    if (control_keys_.empty())
        throw std::invalid_argument("TerminalAccelerationGyroFactor requires at least one control");
    if (dt_ <= 0.0)
        throw std::invalid_argument("TerminalAccelerationGyroFactor requires dt > 0");
}

DynamicsState TerminalAccelerationGyroFactor::propagate(
    const DynamicsState& state, const gtsam::Vector6& control) const {
    const gtsam::Vector3 acceleration = control.head<3>();
    const gtsam::Vector3 angular_speed = control.tail<3>();
    DynamicsState next;
    next.pose = gtsam::Pose3(
        state.pose.rotation() * gtsam::Rot3::Expmap(angular_speed * dt_),
        state.pose.translation() + state.velocity * dt_ + 0.5 * acceleration * dt_ * dt_);
    next.velocity = state.velocity + acceleration * dt_;
    next.body_rate = angular_speed;
    return next;
}

DynamicsState TerminalAccelerationGyroFactor::predict(
    const DynamicsState& initial_state, const std::vector<gtsam::Vector6>& controls) const {
    if (controls.size() != horizon())
        throw std::invalid_argument(
            "Control count must equal TerminalAccelerationGyroFactor horizon");
    DynamicsState predicted = initial_state;
    for (const auto& control : controls) predicted = propagate(predicted, control);
    return predicted;
}

gtsam::Vector TerminalAccelerationGyroFactor::residual(const gtsam::Values& values) const {
    const DynamicsState initial{values.at<gtsam::Pose3>(pose_0_),
                                values.at<gtsam::Vector3>(velocity_0_),
                                gtsam::Vector3::Zero()};
    std::vector<gtsam::Vector6> controls;
    controls.reserve(horizon());
    for (Key key : control_keys_) controls.push_back(values.at<gtsam::Vector6>(key));
    const DynamicsState predicted = predict(initial, controls);
    const gtsam::Pose3& target_pose = values.at<gtsam::Pose3>(pose_j_);

    gtsam::Vector9 error;
    error.segment<3>(0) = target_pose.translation() - predicted.pose.translation();
    error.segment<3>(3) =
        gtsam::Rot3::Logmap(predicted.pose.rotation().between(target_pose.rotation()));
    error.segment<3>(6) = values.at<gtsam::Vector3>(velocity_j_) - predicted.velocity;
    return error;
}

gtsam::Vector TerminalAccelerationGyroFactor::unwhitenedError(
    const gtsam::Values& values, gtsam::OptionalMatrixVecType jacobians) const {
    const gtsam::Vector error = residual(values);
    if (!jacobians) return error;

    jacobians->resize(keys().size());
    auto derivative = [&](Key key, const auto& value) {
        using Value = std::decay_t<decltype(value)>;
        return gtsam::numericalDerivative11<gtsam::Vector, Value>(
            [&](const Value& perturbed_value) {
                gtsam::Values perturbed(values);
                perturbed.update(key, perturbed_value);
                return residual(perturbed);
            }, value);
    };

    std::size_t index = 0;
    (*jacobians)[index++] = derivative(pose_0_, values.at<gtsam::Pose3>(pose_0_));
    (*jacobians)[index++] = derivative(velocity_0_, values.at<gtsam::Vector3>(velocity_0_));
    for (Key key : control_keys_)
        (*jacobians)[index++] = derivative(key, values.at<gtsam::Vector6>(key));
    (*jacobians)[index++] = derivative(pose_j_, values.at<gtsam::Pose3>(pose_j_));
    (*jacobians)[index] = derivative(velocity_j_, values.at<gtsam::Vector3>(velocity_j_));
    return error;
}

TerminalAccelerationGyroMeasurementFactor::TerminalAccelerationGyroMeasurementFactor(
    Key pose_0, Key velocity_0, const gtsam::KeyVector& control_keys,
    const gtsam::Pose3& measured_pose_j, const gtsam::Vector3& measured_velocity_j, double dt,
    const SharedNoiseModel& model)
    : NoiseModelFactor(model, [&] {
          gtsam::KeyVector keys{pose_0, velocity_0};
          keys.insert(keys.end(), control_keys.begin(), control_keys.end());
          return keys;
      }()),
      pose_0_(pose_0), velocity_0_(velocity_0), control_keys_(control_keys),
      measured_pose_j_(measured_pose_j), measured_velocity_j_(measured_velocity_j), dt_(dt) {
    if (!model || model->dim() != 9)
        throw std::invalid_argument(
            "TerminalAccelerationGyroMeasurementFactor requires a 9D Q_j noise model");
    if (control_keys_.empty())
        throw std::invalid_argument(
            "TerminalAccelerationGyroMeasurementFactor requires at least one control");
    if (dt_ <= 0.0)
        throw std::invalid_argument(
            "TerminalAccelerationGyroMeasurementFactor requires dt > 0");
}

DynamicsState TerminalAccelerationGyroMeasurementFactor::propagate(
    const DynamicsState& state, const gtsam::Vector6& control) const {
    const gtsam::Vector3 acceleration = control.head<3>();
    const gtsam::Vector3 angular_speed = control.tail<3>();
    DynamicsState next;
    next.pose = gtsam::Pose3(
        state.pose.rotation() * gtsam::Rot3::Expmap(angular_speed * dt_),
        state.pose.translation() + state.velocity * dt_ + 0.5 * acceleration * dt_ * dt_);
    next.velocity = state.velocity + acceleration * dt_;
    next.body_rate = angular_speed;
    return next;
}

DynamicsState TerminalAccelerationGyroMeasurementFactor::predict(
    const DynamicsState& initial_state, const std::vector<gtsam::Vector6>& controls) const {
    if (controls.size() != horizon())
        throw std::invalid_argument(
            "Control count must equal TerminalAccelerationGyroMeasurementFactor horizon");
    DynamicsState predicted = initial_state;
    for (const auto& control : controls) predicted = propagate(predicted, control);
    return predicted;
}

gtsam::Vector TerminalAccelerationGyroMeasurementFactor::residual(
    const gtsam::Values& values) const {
    const DynamicsState initial{values.at<gtsam::Pose3>(pose_0_),
                                values.at<gtsam::Vector3>(velocity_0_),
                                gtsam::Vector3::Zero()};
    std::vector<gtsam::Vector6> controls;
    controls.reserve(horizon());
    for (Key key : control_keys_) controls.push_back(values.at<gtsam::Vector6>(key));
    const DynamicsState predicted = predict(initial, controls);

    gtsam::Vector9 error;
    error.segment<3>(0) = measured_pose_j_.translation() - predicted.pose.translation();
    error.segment<3>(3) = gtsam::Rot3::Logmap(
        predicted.pose.rotation().between(measured_pose_j_.rotation()));
    error.segment<3>(6) = measured_velocity_j_ - predicted.velocity;
    return error;
}

gtsam::Vector TerminalAccelerationGyroMeasurementFactor::unwhitenedError(
    const gtsam::Values& values, gtsam::OptionalMatrixVecType jacobians) const {
    if (!jacobians) return residual(values);

    const gtsam::Pose3& pose_0 = values.at<gtsam::Pose3>(pose_0_);
    const gtsam::Vector3& velocity_0 = values.at<gtsam::Vector3>(velocity_0_);
    std::vector<gtsam::Vector6> controls;
    controls.reserve(horizon());
    for (Key key : control_keys_) controls.push_back(values.at<gtsam::Vector6>(key));
    const DynamicsState predicted =
        predict({pose_0, velocity_0, gtsam::Vector3::Zero()}, controls);

    gtsam::Matrix3 between_predicted;
    const gtsam::Rot3 relative_rotation = predicted.pose.rotation().between(
        measured_pose_j_.rotation(), between_predicted);
    gtsam::Matrix3 log_relative;
    const gtsam::Vector3 rotation_error =
        gtsam::Rot3::Logmap(relative_rotation, log_relative);
    const gtsam::Matrix3 rotation_error_predicted = log_relative * between_predicted;

    gtsam::Vector9 error;
    error.segment<3>(0) = measured_pose_j_.translation() - predicted.pose.translation();
    error.segment<3>(3) = rotation_error;
    error.segment<3>(6) = measured_velocity_j_ - predicted.velocity;

    // R_j = R_0 E_0 ... E_{j-1}. A perturbation before a rotation suffix is
    // transported into R_j's local frame by the inverse adjoint of that suffix.
    std::vector<gtsam::Rot3> increments(horizon());
    std::vector<gtsam::Matrix3> increment_jacobians(horizon());
    std::vector<gtsam::Rot3> suffix_after(horizon());
    for (std::size_t k = 0; k < horizon(); ++k) {
        increments[k] = gtsam::Rot3::Expmap(controls[k].tail<3>() * dt_,
                                             increment_jacobians[k]);
        increment_jacobians[k] *= dt_;
    }
    gtsam::Rot3 complete_suffix;
    for (std::size_t k = horizon(); k-- > 0;) {
        suffix_after[k] = complete_suffix;
        complete_suffix = increments[k] * complete_suffix;
    }

    jacobians->assign(keys().size(), gtsam::Matrix());
    gtsam::Matrix& pose_jacobian = (*jacobians)[0];
    pose_jacobian = gtsam::Matrix::Zero(9, 6);
    pose_jacobian.block<3, 3>(0, 3) = -pose_0.rotation().matrix();
    pose_jacobian.block<3, 3>(3, 0) =
        rotation_error_predicted * complete_suffix.matrix().transpose();

    gtsam::Matrix& velocity_jacobian = (*jacobians)[1];
    velocity_jacobian = gtsam::Matrix::Zero(9, 3);
    const double duration = horizon() * dt_;
    velocity_jacobian.block<3, 3>(0, 0) = -duration * gtsam::Matrix3::Identity();
    velocity_jacobian.block<3, 3>(6, 0) = -gtsam::Matrix3::Identity();

    for (std::size_t k = 0; k < horizon(); ++k) {
        gtsam::Matrix& control_jacobian = (*jacobians)[k + 2];
        control_jacobian = gtsam::Matrix::Zero(9, 6);
        const double position_coefficient =
            (static_cast<double>(horizon() - k) - 0.5) * dt_ * dt_;
        control_jacobian.block<3, 3>(0, 0) =
            -position_coefficient * gtsam::Matrix3::Identity();
        control_jacobian.block<3, 3>(3, 3) =
            rotation_error_predicted * suffix_after[k].matrix().transpose() *
            increment_jacobians[k];
        control_jacobian.block<3, 3>(6, 0) = -dt_ * gtsam::Matrix3::Identity();
    }
    return error;
}

DynamicsFactor::DynamicsFactor(Key p_i, Key vel_i, Key omega_i, Key input_i, Key p_j, Key vel_j,
                               Key omega_j, float dt, const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j), dt_(dt) {}

DynamicFactor::DynamicFactor(Key p_i, Key vel_i, Key omega_i, Key p_j, Key vel_j, Key omega_j,
                             Key input_i, float dt, float mass, gtsam::Vector3 inertia,
                             gtsam::Vector3 rotor_pos, gtsam::Vector3 drag_k, double ctt,
                             double kmm, const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, omega_i, p_j, vel_j, omega_j, input_i), dt_(dt), mass_(mass),
      rot_inertia_(inertia), rotor_pos_(rotor_pos), drag_k_(drag_k), ct(ctt), km(kmm) {}

gtsam::Vector6 DynamicFactor::Thrust_Torque(const gtsam::Vector4& rpm, const double& ct,
                                            const double& km, const gtsam::Vector3& rotor_pos,
                                            gtsam::Matrix64& Jac) const {
    gtsam::Vector4 rpm_square;

    for (uint i = 0; i < 4u; i++) {
        double rel_pwm = rpm(i);
        rpm_square(i) = rel_pwm * rel_pwm;
    }

    gtsam::Vector3 _thrust = ct * axis;

    gtsam::Vector3 rp1, rp2, rp3, rp4;
    rp1 = rk1_ * rotor_pos;
    gtsam::Vector3 _torque1 = -ct * axis_mat * rp1 + ct * km * axis; // torques of 1rd rotor
    rp2 = rk2_ * rotor_pos;
    gtsam::Vector3 _torque2 = -ct * axis_mat * rp2 - ct * km * axis; // torques of 2rd rotor
    rp3 = rk3_ * rotor_pos;
    gtsam::Vector3 _torque3 = -ct * axis_mat * rp3 + ct * km * axis; // torques of 3rd rotor
    rp4 = rk4_ * rotor_pos;
    gtsam::Vector3 _torque4 = -ct * axis_mat * rp4 - ct * km * axis; // torques of 4rd rotor

    gtsam::Matrix64 effectiveness_matrix;
    effectiveness_matrix.setZero();

    for (uint i = 0; i < 4u; i++) {
        effectiveness_matrix.block(0, i, 3, 1) = _thrust;
    }

    effectiveness_matrix.block(3, 0, 3, 1) = _torque1;
    effectiveness_matrix.block(3, 1, 3, 1) = _torque2;
    effectiveness_matrix.block(3, 2, 3, 1) = _torque3;
    effectiveness_matrix.block(3, 3, 3, 1) = _torque4;

    IPN_LOG_DEBUG << "Rotor effectiveness matrix normalized by thrust coefficient: \n"
                  << effectiveness_matrix / ct;
    gtsam::Vector6 thrust_torque = effectiveness_matrix * rpm_square;

    gtsam::Matrix44 _J;
    _J.setZero();
    _J.diagonal() << rpm;

    Jac = effectiveness_matrix * 2 * _J;

    return thrust_torque;
}

DynamicsFactorTGyro::DynamicsFactorTGyro(Key p_i, Key vel_i, Key input_i, Key p_j, Key vel_j,
                                         float dt, double mass, gtsam::Vector3 drag_k,
                                         const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, input_i, p_j, vel_j), drag_k_(drag_k), mass_(mass), dt_(dt) {}

Vector DynamicsFactorTGyro::evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                                          const gtsam::Vector4& input_i, const gtsam::Pose3& pos_j,
                                          const gtsam::Vector3& vel_j, gtsam::OptionalMatrixType H1,
                                          gtsam::OptionalMatrixType H2,
                                          gtsam::OptionalMatrixType H3,
                                          gtsam::OptionalMatrixType H4,
                                          gtsam::OptionalMatrixType H5) const {
    gtsam::Vector9 err;

    Matrix36 jac_t_posei, jac_t_posej;
    Matrix36 jac_r_posei, jac_r_posej;

    const Point3 p_w_mi = pos_i.translation(jac_t_posei);
    const Rot3 r_w_mi = pos_i.rotation(jac_r_posei);
    const Point3 p_w_mj = pos_j.translation(jac_t_posej);
    const Rot3 r_w_mj = pos_j.rotation(jac_r_posej);

    const double dt_squared = dt_ * dt_;
    const gtsam::Matrix33 world_to_body = r_w_mi.inverse().matrix();

    // position rotation velocity angular_speed error
    gtsam::Matrix33 J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
    // J_rerr_rbj, J_rbi;
    gtsam::Matrix33 J_ri, J_rj, J_dr;

    gtsam::Vector3 pos_err =
        mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5 * gI_ * dt_squared - p_w_mi,
                                J_pe_roti) -
        0.5 * kBodyZ * input_i(0) * dt_squared;
    // gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - 0.5f *
    // (omega_i + omega_j) * dt_; This would cause the heavy fluctuation of omega
    gtsam::Vector3 rot_err =
        Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - input_i.tail(3) * dt_;

    const gtsam::Matrix3 drag_matrix = diagonalMatrix(drag_k_);

    gtsam::Vector3 vel_err =
        mass_ * r_w_mi.unrotate(vel_j - vel_i + gI_ * dt_, J_ve_rot1) -
        kBodyZ * input_i(0) * dt_ -
        drag_matrix * r_w_mi.unrotate(vel_i, J_dv_rit, J_dv_v) * dt_; // - dT * dt_;

    Matrix96 J_e_pi, J_e_posej;

    if (H1) {
        Matrix33 Jac_perr_p = -mass_ * world_to_body;
        Matrix33 Jac_perr_r = mass_ * J_pe_roti;
        Matrix33 Jac_rerr_r = J_dr * J_ri;
        Matrix33 Jac_verr_r =
            mass_ * J_ve_rot1 - drag_matrix * J_dv_rit * dt_; // - A_mat * J_da_ri * dt_;

        Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
        Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
        Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

        J_e_pi.setZero();
        J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
        J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
        J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

        *H1 = J_e_pi;
    }

    if (H2) {
        Matrix93 J_e_v;
        J_e_v.setZero();
        Matrix33 Jac_perr_veli = -mass_ * world_to_body * dt_;
        Matrix33 Jac_verr_v = -mass_ * world_to_body;
        J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
        J_e_v.block(6, 0, 3, 3) = Jac_verr_v - drag_matrix * dt_ * J_dv_v;

        *H2 = J_e_v;
    }

    if (H4) {
        J_e_posej.setZero();
        J_e_posej.block(0, 0, 3, 6) = mass_ * world_to_body * jac_t_posej;
        J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
        *H4 = J_e_posej;
    }

    if (H5) {
        Matrix93 J_e_vj;
        J_e_vj.setZero();
        J_e_vj.block(6, 0, 3, 3) = mass_ * world_to_body;
        *H5 = J_e_vj;
    }

    if (H3) {
        Matrix94 J_T_gyro;
        J_T_gyro.setZero();
        J_T_gyro.block(0, 0, 3, 1) = -0.5 * kBodyZ * dt_squared;
        J_T_gyro.block(3, 1, 3, 3) = -gtsam::I_3x3 * dt_;
        J_T_gyro.block(6, 0, 3, 1) = -kBodyZ * dt_;

        *H3 = J_T_gyro;
    }

    err.head(3) = pos_err;
    err.block(3, 0, 3, 1) = rot_err;
    err.block(6, 0, 3, 1) = vel_err;

    return err;
}

DynamicsFactorTGyroWall::DynamicsFactorTGyroWall(Key p_i, Key vel_i, Key input_i, Key p_j,
                                                 Key vel_j, float dt, double mass,
                                                 gtsam::Vector3 drag_k,
                                                 const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, input_i, p_j, vel_j), drag_k_(drag_k), mass_(mass), dt_(dt) {}

Vector DynamicsFactorTGyroWall::evaluateError(
    const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i, const gtsam::Vector4& input_i,
    const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4,
    gtsam::OptionalMatrixType H5) const {
    gtsam::Vector9 err;

    Matrix36 jac_t_posei, jac_t_posej;
    Matrix36 jac_r_posei, jac_r_posej;

    const Point3 p_w_mi = pos_i.translation(jac_t_posei);
    const Rot3 r_w_mi = pos_i.rotation(jac_r_posei);
    const Point3 p_w_mj = pos_j.translation(jac_t_posej);
    const Rot3 r_w_mj = pos_j.rotation(jac_r_posej);

    const double dt_squared = dt_ * dt_;
    const gtsam::Matrix33 world_to_body = r_w_mi.inverse().matrix();
    constexpr double kWallStiffness = 10.0;
    const gtsam::Vector3 wall_force(0.0, kWallStiffness * (0.05 + p_w_mi.y()), 0.0);
    gtsam::Matrix3 wall_force_jacobian = gtsam::Matrix3::Zero();
    wall_force_jacobian(1, 1) = kWallStiffness;

    // position rotation velocity angular_speed error
    gtsam::Matrix33 J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
    // J_rerr_rbj, J_rbi;
    gtsam::Matrix33 J_ri, J_rj, J_dr;

    gtsam::Vector3 pos_err =
        mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5 * gI_ * dt_squared -
                                    wall_force * dt_squared - p_w_mi,
                                J_pe_roti) -
        0.5 * kBodyZ * input_i(0) * dt_squared;
    // gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - 0.5f *
    // (omega_i + omega_j) * dt_; This would cause the heavy fluctuation of omega

    gtsam::Vector3 rot_err =
        Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - input_i.tail(3) * dt_;

    const gtsam::Matrix3 drag_matrix = diagonalMatrix(drag_k_);

    gtsam::Vector3 vel_err = mass_ * r_w_mi.unrotate(vel_j - vel_i + gI_ * dt_, J_ve_rot1) -
                             kBodyZ * input_i(0) * dt_ -
                             drag_matrix * r_w_mi.unrotate(vel_i, J_dv_rit, J_dv_v) * dt_ -
                             wall_force * dt_;

    Matrix96 J_e_pi, J_e_posej;

    if (H1) {
        Matrix33 Jac_perr_p =
            -mass_ * world_to_body * (gtsam::I_3x3 + wall_force_jacobian * dt_squared);
        Matrix33 Jac_perr_r = mass_ * J_pe_roti;
        Matrix33 Jac_rerr_r = J_dr * J_ri;
        Matrix33 Jac_verr_r =
            mass_ * J_ve_rot1 - drag_matrix * J_dv_rit * dt_; // - A_mat * J_da_ri * dt_;

        Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
        Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
        Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei -
                                  wall_force_jacobian * dt_ * jac_t_posei;

        J_e_pi.setZero();
        J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
        J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
        J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

        *H1 = J_e_pi;
    }

    if (H2) {
        Matrix93 J_e_v;
        J_e_v.setZero();
        Matrix33 Jac_perr_veli = -mass_ * world_to_body * dt_;
        Matrix33 Jac_verr_v = -mass_ * world_to_body;
        J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
        J_e_v.block(6, 0, 3, 3) = Jac_verr_v - drag_matrix * dt_ * J_dv_v;

        *H2 = J_e_v;
    }

    if (H4) {
        J_e_posej.setZero();
        J_e_posej.block(0, 0, 3, 6) = mass_ * world_to_body * jac_t_posej;
        J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
        *H4 = J_e_posej;
    }

    if (H5) {
        Matrix93 J_e_vj;
        J_e_vj.setZero();
        J_e_vj.block(6, 0, 3, 3) = mass_ * world_to_body;
        *H5 = J_e_vj;
    }

    if (H3) {
        Matrix94 J_T_gyro;
        J_T_gyro.setZero();
        J_T_gyro.block(0, 0, 3, 1) = -0.5 * kBodyZ * dt_squared;
        J_T_gyro.block(3, 1, 3, 3) = -gtsam::I_3x3 * dt_;
        J_T_gyro.block(6, 0, 3, 1) = -kBodyZ * dt_;

        *H3 = J_T_gyro;
    }

    err.head(3) = pos_err;
    err.block(3, 0, 3, 1) = rot_err;
    err.block(6, 0, 3, 1) = vel_err;

    return err;
}

gtsam::Vector6 DynamicFactor::Thrust_Torque(const gtsam::Vector4& rpm_square, const double& ct,
                                            const double& km, const gtsam::Vector3& rotor_pos,
                                            gtsam::Vector3& A) const {
    gtsam::Vector3 _thrust = ct * axis;

    gtsam::Vector3 rp1, rp2, rp3, rp4;
    rp1 = rk1_ * rotor_pos;
    gtsam::Vector3 _torque1 = -ct * axis_mat * rp1 + ct * km * axis; // torques of 1rd rotor
    rp2 = rk2_ * rotor_pos;
    gtsam::Vector3 _torque2 = -ct * axis_mat * rp2 - ct * km * axis; // torques of 2rd rotor
    rp3 = rk3_ * rotor_pos;
    gtsam::Vector3 _torque3 = -ct * axis_mat * rp3 + ct * km * axis; // torques of 3rd rotor
    rp4 = rk4_ * rotor_pos;
    gtsam::Vector3 _torque4 = -ct * axis_mat * rp4 - ct * km * axis; // torques of 4rd rotor

    gtsam::Matrix64 effectiveness_matrix;
    effectiveness_matrix.setZero();

    for (uint i = 0; i < 4u; i++) {
        effectiveness_matrix.block(0, i, 3, 1) = _thrust;
    }

    effectiveness_matrix.block(3, 0, 3, 1) = _torque1;
    effectiveness_matrix.block(3, 1, 3, 1) = _torque2;
    effectiveness_matrix.block(3, 2, 3, 1) = _torque3;
    effectiveness_matrix.block(3, 3, 3, 1) = _torque4;

    // IPN_LOG_DEBUG << "Rotor effectiveness matrix normalized by thrust coefficient: \n" <<
    // effectiveness_matrix / ct;
    gtsam::Vector6 thrust_torque = effectiveness_matrix * rpm_square;

    gtsam::Matrix33 F_mat;
    F_mat.setZero();
    gtsam::Vector3 f_v(thrust_torque[2], thrust_torque[2], 0);
    F_mat.diagonal() << f_v;
    gtsam::Vector3 torque_bias_mass = F_mat * A;
    thrust_torque.tail(3) = thrust_torque.tail(3) - torque_bias_mass;
    return thrust_torque;
}

Vector DynamicFactor::evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                                    const gtsam::Vector3& omega_i, const gtsam::Pose3& pos_j,
                                    const gtsam::Vector3& vel_j, const gtsam::Vector3& omega_j,
                                    const gtsam::Vector4& input_i, gtsam::OptionalMatrixType H1,
                                    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                                    gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5,
                                    gtsam::OptionalMatrixType H6,
                                    gtsam::OptionalMatrixType H7) const {
    gtsam::Vector12 err;
    gtsam::Matrix64 J_tt_rpm;

    gtsam::Vector6 thrust_torque = Thrust_Torque(input_i, ct, km, rotor_pos_, J_tt_rpm);

    Matrix36 jac_t_posei, jac_t_posej;
    Matrix36 jac_r_posei, jac_r_posej;

    const Point3 p_w_mi = pos_i.translation(jac_t_posei);
    const Rot3 r_w_mi = pos_i.rotation(jac_r_posei);
    const Point3 p_w_mj = pos_j.translation(jac_t_posej);
    const Rot3 r_w_mj = pos_j.rotation(jac_r_posej);

    double dtt = dt_ * dt_;
    double Ix = rot_inertia_.x();
    double Iy = rot_inertia_.y();
    double Iz = rot_inertia_.z();
    gtsam::Matrix3 J, J_inv;
    J << Ix, 0, 0, 0, Iy, 0, 0, 0, Iz;
    J_inv << 1.0f / Ix, 0, 0, 0, 1.0f / Iy, 0, 0, 0, 1.0f / Iz;
    gtsam::Matrix33 _unrbi_matrix = r_w_mi.inverse().matrix();

    // position rotation velocity angular_speed error
    gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
    // J_rerr_rbj, J_rbi;
    gtsam::Matrix33 J_ri, J_rj, J_dr;

    gtsam::Vector3 pos_err =
        mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f * gI_ * dtt - p_w_mi, J_pe_roti) -
        0.5f * thrust_torque.head(3) * dtt;
    // gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - 0.5f *
    // (omega_i + omega_j) * dt_; This would cause the heavy fluctuation of omega

    gtsam::Vector3 rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - omega_i * dt_;

    gtsam::Matrix3 drag_matrix;
    drag_matrix.setZero();
    drag_matrix.diagonal() << drag_k_;

    gtsam::Vector3 vel_err =
        mass_ * r_w_mi.unrotate(vel_j - vel_i + gI_ * dt_, J_ve_rot1) -
        thrust_torque.head(3) * dt_ -
        drag_matrix * r_w_mi.unrotate(vel_i, J_dv_rit, J_dv_v) * dt_; // - dT * dt_;

    gtsam::Vector3 asp_err = J * (omega_j - omega_i) + skewSymmetric(omega_i) * J * omega_i * dt_ -
                             thrust_torque.tail(3) * dt_;

    Matrix126 J_e_pi, J_e_posej;

    if (H1) {
        Matrix33 Jac_perr_p = -mass_ * _unrbi_matrix;
        Matrix33 Jac_perr_r = mass_ * J_pe_roti;
        Matrix33 Jac_rerr_r = J_dr * J_ri;
        Matrix33 Jac_verr_r =
            mass_ * J_ve_rot1 - drag_matrix * J_dv_rit * dt_; // - A_mat * J_da_ri * dt_;

        Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
        Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
        Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

        J_e_pi.setZero();
        J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
        J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
        J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

        *H1 = J_e_pi;
    }

    if (H2) {
        Matrix123 J_e_v;
        J_e_v.setZero();
        Matrix33 Jac_perr_veli = -mass_ * _unrbi_matrix * dt_;
        Matrix33 Jac_verr_v = -mass_ * _unrbi_matrix;
        J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
        J_e_v.block(6, 0, 3, 3) = Jac_verr_v - drag_matrix * dt_ * J_dv_v;

        *H2 = J_e_v;
    }

    if (H3) {
        Matrix123 J_e_omage;
        J_e_omage.setZero();
        double a = Iz - Iy;
        double b = Ix - Iz;
        double c = Iy - Ix;

        Matrix3 d_omega;
        d_omega << 0, a * omega_i[2], a * omega_i[1], b * omega_i[2], 0, b * omega_i[0],
            c * omega_i[1], c * omega_i[0], 0;

        Matrix33 Jac_r_omega =
            -gtsam::I_3x3 *
            dt_; // - 0.5f * gtsam::I_3x3 * dt_; // SO3::ExpmapDerivative(omega_i * dt_) * dt_;
        Matrix33 Jac_omega_omega = -J + d_omega * dt_;
        J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
        J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

        *H3 = J_e_omage;
    }

    if (H4) {
        J_e_posej.setZero();
        J_e_posej.block(0, 0, 3, 6) = mass_ * _unrbi_matrix * jac_t_posej;
        J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
        *H4 = J_e_posej;
    }

    if (H5) {
        Matrix123 J_e_vj;
        J_e_vj.setZero();
        J_e_vj.block(6, 0, 3, 3) = mass_ * _unrbi_matrix;
        *H5 = J_e_vj;
    }

    if (H6) {
        Matrix123 J_e_omegaj;
        J_e_omegaj.setZero();
        // J_e_omegaj.block(3, 0, 3, 3) = - 0.5f * gtsam::I_3x3 * dt_;
        J_e_omegaj.block(9, 0, 3, 3) = J;
        *H6 = J_e_omegaj;
    }

    Matrix126 J_e_thrust_torques;
    Matrix123 J_torques;

    J_e_thrust_torques.setZero();
    J_e_thrust_torques.block(6, 0, 3, 3) = -I_3x3 * dt_; // - J_dT_T * dt_;
    J_e_thrust_torques.block(9, 3, 3, 3) = -I_3x3 * dt_;

    J_e_thrust_torques.block(0, 0, 3, 3) = -I_3x3 * dtt * 0.5f;

    J_torques.setZero();
    J_torques.block(9, 0, 3, 3) = -I_3x3 * dt_;

    if (H7) {
        *H7 = J_e_thrust_torques * J_tt_rpm;
    }

    err.head(3) = pos_err;
    err.block(3, 0, 3, 1) = rot_err;
    err.block(6, 0, 3, 1) = vel_err;
    err.tail(3) = asp_err;

    return err;
}

Vector DynamicsFactor::evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                                     const gtsam::Vector3& omega_i, const gtsam::Vector4& input_i,
                                     const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j,
                                     const gtsam::Vector3& omega_j, gtsam::OptionalMatrixType H1,
                                     gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                                     gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5,
                                     gtsam::OptionalMatrixType H6,
                                     gtsam::OptionalMatrixType H7) const {
    gtsam::Vector12 err;
    Matrix36 jac_t_posei, jac_t_posej;
    Matrix36 jac_r_posei, jac_r_posej;

    const Point3 p_w_i = pos_i.translation(jac_t_posei);
    const Rot3 r_w_bi = pos_i.rotation(jac_r_posei);
    const Point3 p_w_j = pos_j.translation(jac_t_posej);
    const Rot3 r_w_bj = pos_j.rotation(jac_r_posej);

    // force and torque
    Matrix4 K1, K2;
    K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, 0,
        0, dynamics_params_.arm_length * dynamics_params_.k_f,
        -dynamics_params_.arm_length * dynamics_params_.k_f,
        -dynamics_params_.arm_length * dynamics_params_.k_f,
        dynamics_params_.arm_length * dynamics_params_.k_f, 0, 0, dynamics_params_.k_m,
        dynamics_params_.k_m, -dynamics_params_.k_m, -dynamics_params_.k_m;

    K2 << 2.0 * input_i[0], 0, 0, 0, 0, 2.0 * input_i[1], 0, 0, 0, 0, 2.0 * input_i[2], 0, 0, 0, 0,
        2.0 * input_i[3];

    gtsam::Vector4 input2(input_i[0] * input_i[0], input_i[1] * input_i[1], input_i[2] * input_i[2],
                          input_i[3] * input_i[3]);
    gtsam::Vector4 T_mb = K1 * input2;

    // position rotation velocity error
    gtsam::Vector3 p_err = p_w_j - (p_w_i + vel_i * dt_);
    // IPN_LOG_DEBUG << "p_1:" << p_w_i;
    // IPN_LOG_DEBUG << "p_2:" << p_w_j;
    // IPN_LOG_DEBUG << "v_1:" << vel_i;
    // IPN_LOG_DEBUG << "p_error:" << p_err;

    gtsam::Matrix33 J_rerr_rbj, J_rbi;
    gtsam::Vector3 rot_err = Rot3::Logmap(
        r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
    gtsam::Vector3 vel_err =
        vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) +
                          r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))) *
                             dt_);

    // IPN_LOG_DEBUG << "v_error:" << vel_err;
    // IPN_LOG_DEBUG << "a_sum:" << -gtsam::Vector3(0, 0, dynamics_params_.g) +
    // r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass));

    // IPN_LOG_DEBUG << "Dynmaics Factor p_err: \n" << p_err;
    // IPN_LOG_DEBUG << "Dynmaics Factor r_err: \n" << rot_err;
    // IPN_LOG_DEBUG << "Dynmaics Factor v_err: \n" << vel_err;

    // omage error
    gtsam::Matrix3 J, J_inv;
    J << dynamics_params_.Ixx, 0, 0, 0, dynamics_params_.Iyy, 0, 0, 0, dynamics_params_.Izz;
    J_inv << 1.0 / dynamics_params_.Ixx, 0, 0, 0, 1.0 / dynamics_params_.Iyy, 0, 0, 0,
        1.0 / dynamics_params_.Izz;
    gtsam::Vector3 omega_err =
        omega_j - omega_i - J_inv * (T_mb.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;
    // IPN_LOG_DEBUG << "Dynmaics Factor omage_err: \n" << omega_err;

    if (H1) {
        Matrix33 Jac_perr_p = -Matrix33::Identity();
        Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
        Matrix33 Jac_verr_r = r_w_bi.matrix() *
                              skewSymmetric(gtsam::Vector3(0, 0, T_mb[0] / dynamics_params_.mass)) *
                              dt_;

        Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei;
        Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
        Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;
        // IPN_LOG_DEBUG << Jac_perr_posei;
        // IPN_LOG_DEBUG << Jac_rerr_posei;
        // IPN_LOG_DEBUG << Jac_verr_posei;

        Matrix126 J_e_pi;
        J_e_pi.setZero();
        J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
        J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
        J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

        *H1 = J_e_pi;

        // IPN_LOG_DEBUG << "*H1: \n" << *H1;
    }
    if (H2) {
        Matrix123 J_e_v;
        J_e_v.setZero();
        Matrix33 Jac_perr_veli = -Matrix33::Identity() * dt_;
        Matrix33 Jac_verr_v = -Matrix33::Identity();
        J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
        J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

        *H2 = J_e_v;
        // IPN_LOG_DEBUG << "*H2: \n" << *H2;
    }
    if (H3) {
        Matrix123 J_e_omage;
        J_e_omage.setZero();
        double a = 1.0f / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
        double b = 1.0f / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
        double c = 1.0f / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);
        Matrix3 d_omega;
        d_omega << -1, a * omega_i[2], a * omega_i[1], b * omega_i[2], -1, b * omega_i[0],
            c * omega_i[1], c * omega_i[0], -1;

        Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
        Matrix33 Jac_omega_omega = d_omega * dt_;
        J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
        J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

        *H3 = J_e_omage;
        // IPN_LOG_DEBUG << "*H3: \n" << *H3;
    }
    if (H4) {
        Matrix124 J_e_input;
        J_e_input.setZero();
        Matrix124 _B;
        _B.setZero();
        _B.block(6, 0, 3, 1) =
            r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0f / dynamics_params_.mass) * dt_;
        _B.block(9, 1, 3, 3) = J_inv * dt_;

        J_e_input = -_B * K1 * K2;
        *H4 = J_e_input;
        // IPN_LOG_DEBUG << "*H4: \n" << *H4;
    }
    if (H5) {
        Matrix126 J_e_posej;
        J_e_posej.setZero();
        J_e_posej.block(0, 0, 3, 6) = jac_t_posej;
        J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;

        *H5 = J_e_posej;
        // IPN_LOG_DEBUG << "*H5: \n" << *H5;
    }
    if (H6) {
        Matrix123 J_e_vj;
        J_e_vj.setZero();
        J_e_vj.block(6, 0, 3, 3) = Matrix33::Identity();
        *H6 = J_e_vj;
        // IPN_LOG_DEBUG << "*H6: \n" << *H6;
    }
    if (H7) {
        Matrix123 J_e_omagej;

        J_e_omagej.setZero();
        J_e_omagej.block(9, 0, 3, 3) = Matrix33::Identity();
        *H7 = J_e_omagej;
        // IPN_LOG_DEBUG << "*H7: \n" << *H7;
    }

    err.head(3) = p_err;
    err.block(3, 0, 3, 1) = rot_err;
    err.block(6, 0, 3, 1) = vel_err;
    err.tail(3) = omega_err;
    // IPN_LOG_DEBUG << "factor error: " << err.transpose();
    return err;
}

DynamicsFactorTm::DynamicsFactorTm(Key p_i, Key vel_i, Key omega_i, Key input_i, Key p_j, Key vel_j,
                                   Key omega_j, float dt, const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j), dt_(dt) {}

Vector DynamicsFactorTm::evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                                       const gtsam::Vector3& omega_i, const gtsam::Vector4& input_i,
                                       const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j,
                                       const gtsam::Vector3& omega_j, gtsam::OptionalMatrixType H1,
                                       gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                                       gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5,
                                       gtsam::OptionalMatrixType H6,
                                       gtsam::OptionalMatrixType H7) const {
    gtsam::Vector12 err;
    Matrix36 jac_t_posei, jac_t_posej;
    Matrix36 jac_r_posei, jac_r_posej;

    const Point3 p_w_i = pos_i.translation(jac_t_posei);
    const Rot3 r_w_bi = pos_i.rotation(jac_r_posei);
    const Point3 p_w_j = pos_j.translation(jac_t_posej);
    const Rot3 r_w_bj = pos_j.rotation(jac_r_posej);

    // force and torque

    // position rotation velocity error
    gtsam::Vector3 p_err =
        p_w_j - p_w_i - vel_i * dt_ -
        0.5 *
            (-gtsam::Vector3(0, 0, dynamics_params_.g) +
             r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))) *
            dt_ * dt_;
    // IPN_LOG_DEBUG << "p_1:" << p_w_i;
    // IPN_LOG_DEBUG << "p_2:" << p_w_j;
    // IPN_LOG_DEBUG << "v_1:" << vel_i;
    // IPN_LOG_DEBUG << "p_error:" << p_err;

    gtsam::Matrix33 J_rerr_rbj, J_rbi;
    gtsam::Vector3 rot_err = Rot3::Logmap(
        r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
    gtsam::Vector3 vel_err =
        vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) +
                          r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))) *
                             dt_);

    // IPN_LOG_DEBUG << "v_error:" << vel_err;
    // IPN_LOG_DEBUG << "a_sum:" << -gtsam::Vector3(0, 0, dynamics_params_.g) +
    //    r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass));

    // // IPN_LOG_DEBUG << "Dynmaics Factor p_err: \n" << p_err;
    // IPN_LOG_DEBUG << "Dynmaics Factor r_err: \n" << rot_err;
    // IPN_LOG_DEBUG << "Dynmaics Factor v_err: \n" << vel_err;

    // omage error
    gtsam::Matrix3 J, J_inv;
    J << dynamics_params_.Ixx, 0, 0, 0, dynamics_params_.Iyy, 0, 0, 0, dynamics_params_.Izz;
    J_inv << 1.0 / dynamics_params_.Ixx, 0, 0, 0, 1.0 / dynamics_params_.Iyy, 0, 0, 0,
        1.0 / dynamics_params_.Izz;
    gtsam::Vector3 omega_err =
        omega_j - omega_i - J_inv * (input_i.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;
    // IPN_LOG_DEBUG << "Dynmaics Factor omage_err: \n" << omega_err;

    if (H1) {
        Matrix33 Jac_perr_p = -Matrix33::Identity();
        Matrix33 Jac_perr_r =
            r_w_bi.matrix() *
            gtsam::skewSymmetric(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)) * dt_ *
            dt_ * 0.5f;
        Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
        Matrix33 Jac_verr_r =
            r_w_bi.matrix() *
            skewSymmetric(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)) * dt_;

        Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
        Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
        Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;
        // IPN_LOG_DEBUG << Jac_perr_posei;
        // IPN_LOG_DEBUG << Jac_rerr_posei;
        // IPN_LOG_DEBUG << Jac_verr_posei;

        Matrix126 J_e_pi;
        J_e_pi.setZero();
        J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
        J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
        J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

        *H1 = J_e_pi;

        // IPN_LOG_DEBUG << "*H1: \n" << *H1;
    }

    if (H2) {
        Matrix123 J_e_v;
        J_e_v.setZero();
        Matrix33 Jac_perr_veli = -Matrix33::Identity() * dt_;
        Matrix33 Jac_verr_v = -Matrix33::Identity();
        J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
        J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

        *H2 = J_e_v;
        // IPN_LOG_DEBUG << "*H2: \n" << *H2;
    }

    if (H3) {
        Matrix123 J_e_omage;
        J_e_omage.setZero();
        double a = 1.0f / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
        double b = 1.0f / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
        double c = 1.0f / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);

        Matrix3 d_omega;
        d_omega << 0, a * omega_i[2], a * omega_i[1], b * omega_i[2], 0, b * omega_i[0],
            c * omega_i[1], c * omega_i[0], 0;

        Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
        Matrix33 Jac_omega_omega = -I_3x3 + d_omega * dt_;
        J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
        J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

        *H3 = J_e_omage;
        // IPN_LOG_DEBUG << "*H3: \n" << *H3;
    }

    if (H4) {
        Matrix124 J_e_input;
        J_e_input.setZero();
        Matrix124 _B;
        _B.setZero();

        _B.block(6, 0, 3, 1) =
            r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0f / dynamics_params_.mass) * dt_;
        _B.block(9, 1, 3, 3) = J_inv * dt_;

        J_e_input = -_B;
        J_e_input.block(0, 0, 3, 1) =
            -r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0f / dynamics_params_.mass) * dt_ * dt_ * 0.5;
        *H4 = J_e_input;
        // IPN_LOG_DEBUG << "*H4: \n" << *H4;
    }

    if (H5) {
        Matrix126 J_e_posej;
        J_e_posej.setZero();
        J_e_posej.block(0, 0, 3, 6) = jac_t_posej;
        J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;

        *H5 = J_e_posej;
        // IPN_LOG_DEBUG << "*H5: \n" << *H5;
    }
    if (H6) {
        Matrix123 J_e_vj;
        J_e_vj.setZero();
        J_e_vj.block(6, 0, 3, 3) = Matrix33::Identity();
        *H6 = J_e_vj;
        // IPN_LOG_DEBUG << "*H6: \n" << *H6;
    }

    if (H7) {
        Matrix123 J_e_omagej;

        J_e_omagej.setZero();
        J_e_omagej.block(9, 0, 3, 3) = Matrix33::Identity();
        *H7 = J_e_omagej;
        // IPN_LOG_DEBUG << "*H7: \n" << *H7;
    }

    err.head(3) = p_err;
    err.block(3, 0, 3, 1) = rot_err;
    err.block(6, 0, 3, 1) = vel_err;
    err.tail(3) = omega_err;
    // IPN_LOG_DEBUG << "factor error: " << err.transpose();
    return err;
}

// Dynamics Factor based on thrust and torques.

DynamicsFactorFullTM::DynamicsFactorFullTM(Key p_i, Key vel_i, Key omega_i, Key tm_ij, Key p_j,
                                           Key vel_j, Key omega_j, float dt,
                                           const SharedNoiseModel& model)
    : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j), dt_(dt) {}

Vector DynamicsFactorFullTM::evaluateError(
    const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i, const gtsam::Vector3& omega_i,
    const gtsam::Vector6& thrust_torque, const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j,
    const gtsam::Vector3& omega_j, gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5,
    gtsam::OptionalMatrixType H6, gtsam::OptionalMatrixType H7) const {
    gtsam::Vector12 err;
    Matrix36 jac_t_posei, jac_t_posej;
    Matrix36 jac_r_posei, jac_r_posej;

    const Point3 p_w_i = pos_i.translation(jac_t_posei);
    const Rot3 r_w_bi = pos_i.rotation(jac_r_posei);
    const Point3 p_w_j = pos_j.translation(jac_t_posej);
    const Rot3 r_w_bj = pos_j.rotation(jac_r_posej);

    // position rotation velocity error
    gtsam::Vector3 p_err = p_w_j - (p_w_i + vel_i * dt_);
    gtsam::Matrix33 J_rerr_rbj, J_rbi;
    gtsam::Vector3 rot_err = Rot3::Logmap(
        r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
    gtsam::Vector3 vel_err =
        vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) +
                          r_w_bi.rotate(thrust_torque.head(3) / dynamics_params_.mass)) *
                             dt_);

    // omage error
    gtsam::Matrix3 J, J_inv;
    J << dynamics_params_.Ixx, 0, 0, 0, dynamics_params_.Iyy, 0, 0, 0, dynamics_params_.Izz;
    J_inv << 1.0 / dynamics_params_.Ixx, 0, 0, 0, 1.0 / dynamics_params_.Iyy, 0, 0, 0,
        1.0 / dynamics_params_.Izz;
    gtsam::Vector3 omega_err =
        omega_j - omega_i -
        J_inv * (thrust_torque.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;

    if (H1) {
        Matrix33 Jac_perr_p = -Matrix33::Identity();
        Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
        Matrix33 Jac_verr_r =
            r_w_bi.matrix() * skewSymmetric(thrust_torque.head(3) / dynamics_params_.mass) * dt_;

        Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei;
        Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
        Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

        Matrix126 J_e_pi;
        J_e_pi.setZero();
        J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
        J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
        J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

        *H1 = J_e_pi;
    }

    if (H2) {
        Matrix123 J_e_v;
        J_e_v.setZero();
        Matrix33 Jac_perr_veli = -Matrix33::Identity() * dt_;
        Matrix33 Jac_verr_v = -Matrix33::Identity();
        J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
        J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

        *H2 = J_e_v;
    }

    if (H3) {
        Matrix123 J_e_omage;
        J_e_omage.setZero();
        double a = 1.0 / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
        double b = 1.0 / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
        double c = 1.0 / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);

        Matrix3 d_omega;
        d_omega << 0, a * omega_i[2], a * omega_i[1], b * omega_i[2], 0, b * omega_i[0],
            c * omega_i[1], c * omega_i[0], 0;

        Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
        Matrix33 Jac_omega_omega = -I_3x3 + d_omega * dt_;
        J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
        J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

        *H3 = J_e_omage;
    }

    if (H4) {
        Matrix126 J_e_input;
        J_e_input.setZero();
        Matrix126 _B;
        _B.setZero();
        _B.block(6, 0, 3, 3) =
            r_w_bi.matrix() * gtsam::Matrix3::Identity() / dynamics_params_.mass * dt_;
        _B.block(9, 3, 3, 3) = J_inv * dt_;

        J_e_input = -_B;
        *H4 = J_e_input;
    }

    if (H5) {
        Matrix126 J_e_posej;
        J_e_posej.setZero();
        J_e_posej.block(0, 0, 3, 6) = jac_t_posej;
        J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;

        *H5 = J_e_posej;
    }

    if (H6) {
        Matrix123 J_e_vj;
        J_e_vj.setZero();
        J_e_vj.block(6, 0, 3, 3) = Matrix33::Identity();
        *H6 = J_e_vj;
    }

    if (H7) {
        Matrix123 J_e_omagej;

        J_e_omagej.setZero();
        J_e_omagej.block(9, 0, 3, 3) = Matrix33::Identity();
        *H7 = J_e_omagej;
    }

    err.head(3) = p_err;
    err.block(3, 0, 3, 1) = rot_err;
    err.block(6, 0, 3, 1) = vel_err;
    err.tail(3) = omega_err;

    return err;
}

// Force and torques between factor.
BetForceMoments::BetForceMoments(Key input_i, Key input_j, const SharedNoiseModel& model)
    : Base(model, input_i, input_j) {}

Vector BetForceMoments::evaluateError(const gtsam::Vector4& input_i, const gtsam::Vector4& input_j,
                                      gtsam::OptionalMatrixType H1,
                                      gtsam::OptionalMatrixType H2) const {
    gtsam::Vector4 err;
    err = input_j - input_i;
    if (H1) {
        *H1 = -gtsam::Matrix4::Identity();
    }
    if (H2) {
        *H2 = gtsam::Matrix4::Identity();
    }

    return err;
}

Vector ControlLimitFactor::evaluateError(const gtsam::Vector4& input,
                                         gtsam::OptionalMatrixType H1) const {
    gtsam::Vector4 error;
    gtsam::Matrix4 jac;
    jac.setZero();

    for (uint i = 0; i < 4; i++) {
        if (input[i] >= low_ + thr_ && input[i] <= high_ - thr_) {
            error(i) = 0;
            jac(i, i) = 0;
        } else if (input[i] < low_ + thr_) {
            error(i) = alpha_ * (low_ + thr_ - input[i]);
            jac(i, i) = -alpha_;
        } else {
            error(i) = alpha_ * (input[i] - high_ + thr_);
            jac(i, i) = alpha_;
        }
    }
    if (H1) {
        *H1 = jac;
    }

    return error;
}

Vector ControlLimitTGyroFactor::evaluateError(const gtsam::Vector4& input,
                                              gtsam::OptionalMatrixType H1) const {
    gtsam::Vector4 error;
    gtsam::Matrix4 jac;
    jac.setZero();

    uint i = 0;
    if (input[i] >= T_low_ + T_thr_ && input[i] <= T_high_ - T_thr_) {
        error(i) = 0;
        jac(i, i) = 0;
    } else if (input[i] < T_low_ + T_thr_) {
        error(i) = alpha_ * (T_low_ + T_thr_ - input[i]);
        jac(i, i) = -alpha_;
    } else {
        error(i) = alpha_ * (input[i] - T_high_ + T_thr_);
        jac(i, i) = alpha_;
    }

    for (i = 1; i < 4; i++) {
        if (input[i] >= Gyro_low_ + Gyro_thr_ && input[i] <= Gyro_high_ - Gyro_thr_) {
            error(i) = 0;
            jac(i, i) = 0;
        } else if (input[i] < Gyro_low_ + Gyro_thr_) {
            error(i) = alpha_ * (Gyro_low_ + Gyro_thr_ - input[i]);
            jac(i, i) = -alpha_;
        } else {
            error(i) = alpha_ * (input[i] - Gyro_high_ + Gyro_thr_);
            jac(i, i) = alpha_;
        }
    }
    if (H1) {
        *H1 = jac;
    }

    return error;
}

} // namespace UAVFactor
