#ifndef __DYNAMICS_PLANNING_FACTOR_H__
#define __DYNAMICS_PLANNING_FACTOR_H__

#include <ipn_mpc/control/dynamics_params.h>
#include <ipn_mpc/factors/gtsam_compatibility.h>
#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor {

struct DynamicsState {
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity{gtsam::Vector3::Zero()};
    gtsam::Vector3 body_rate{gtsam::Vector3::Zero()};
};

/** Soft multiple-shooting constraint
 *
 *   e_j = x_j - f^(j)(x_0, u_0, ..., u_{j-1}),
 *   cost = 0.5 e_j^T Q_j e_j.
 *
 * State ordering is [position, rotation, velocity, body rate] (12 DoF), and
 * controls are [collective thrust, tau_x, tau_y, tau_z]. Q_j is represented by
 * the 12-dimensional Gaussian information noise model.
 */
class GTSAM_EXPORT TerminalDynamicsFactor : public gtsam::NoiseModelFactor {
  public:
    using shared_ptr = boost::shared_ptr<TerminalDynamicsFactor>;
    using gtsam::NoiseModelFactor::unwhitenedError;

    TerminalDynamicsFactor() = default;
    TerminalDynamicsFactor(Key pose_0, Key velocity_0, Key body_rate_0,
                           const gtsam::KeyVector& control_keys, Key pose_j, Key velocity_j,
                           Key body_rate_j, double dt, const SharedNoiseModel& model,
                           const DynamicsParams& dynamics_params = DynamicsParams());

    gtsam::Vector unwhitenedError(const gtsam::Values& values,
                                  gtsam::OptionalMatrixVecType jacobians = nullptr) const override;
    DynamicsState predict(const DynamicsState& initial_state,
                          const std::vector<gtsam::Vector4>& controls) const;
    std::size_t horizon() const { return control_keys_.size(); }

  private:
    gtsam::Vector residual(const gtsam::Values& values) const;
    DynamicsState propagate(const DynamicsState& state, const gtsam::Vector4& control) const;

    gtsam::Key pose_0_, velocity_0_, body_rate_0_;
    gtsam::KeyVector control_keys_;
    gtsam::Key pose_j_, velocity_j_, body_rate_j_;
    double dt_{0.0};
    DynamicsParams dynamics_params_;
};

/** Variable-horizon kinematic factor with u_k = [a_world, omega_body].
 *
 *   e_j = [p_j - p_hat_j, Log(R_hat_j^-1 R_j), v_j - v_hat_j],
 *   cost = 0.5 e_j^T Q_j e_j.
 */
class GTSAM_EXPORT TerminalAccelerationGyroFactor : public gtsam::NoiseModelFactor {
  public:
    using shared_ptr = boost::shared_ptr<TerminalAccelerationGyroFactor>;
    using gtsam::NoiseModelFactor::unwhitenedError;

    TerminalAccelerationGyroFactor() = default;
    TerminalAccelerationGyroFactor(Key pose_0, Key velocity_0,
                                   const gtsam::KeyVector& control_keys, Key pose_j,
                                   Key velocity_j, double dt, const SharedNoiseModel& model);

    gtsam::Vector unwhitenedError(const gtsam::Values& values,
                                  gtsam::OptionalMatrixVecType jacobians = nullptr) const override;
    DynamicsState predict(const DynamicsState& initial_state,
                          const std::vector<gtsam::Vector6>& controls) const;
    std::size_t horizon() const { return control_keys_.size(); }

  private:
    gtsam::Vector residual(const gtsam::Values& values) const;
    DynamicsState propagate(const DynamicsState& state, const gtsam::Vector6& control) const;

    gtsam::Key pose_0_, velocity_0_;
    gtsam::KeyVector control_keys_;
    gtsam::Key pose_j_, velocity_j_;
    double dt_{0.0};
};

/** Set-point rollout factor with u_k = [a_world, omega_body].
 *
 * The target is a fixed measurement rather than an optimized x_j variable:
 *   e_j = [p_j^meas - p_hat_j,
 *          Log(R_hat_j^-1 R_j^meas),
 *          v_j^meas - v_hat_j],
 *   cost = 0.5 e_j^T Q_j e_j.
 */
class GTSAM_EXPORT TerminalAccelerationGyroMeasurementFactor
    : public gtsam::NoiseModelFactor {
  public:
    using shared_ptr = boost::shared_ptr<TerminalAccelerationGyroMeasurementFactor>;
    using gtsam::NoiseModelFactor::unwhitenedError;

    TerminalAccelerationGyroMeasurementFactor() = default;
    TerminalAccelerationGyroMeasurementFactor(
        Key pose_0, Key velocity_0, const gtsam::KeyVector& control_keys,
        const gtsam::Pose3& measured_pose_j, const gtsam::Vector3& measured_velocity_j,
        double dt, const SharedNoiseModel& model);

    gtsam::Vector unwhitenedError(const gtsam::Values& values,
                                  gtsam::OptionalMatrixVecType jacobians = nullptr) const override;
    DynamicsState predict(const DynamicsState& initial_state,
                          const std::vector<gtsam::Vector6>& controls) const;
    std::size_t horizon() const { return control_keys_.size(); }

  private:
    gtsam::Vector residual(const gtsam::Values& values) const;
    DynamicsState propagate(const DynamicsState& state, const gtsam::Vector6& control) const;

    gtsam::Key pose_0_, velocity_0_;
    gtsam::KeyVector control_keys_;
    gtsam::Pose3 measured_pose_j_;
    gtsam::Vector3 measured_velocity_j_{gtsam::Vector3::Zero()};
    double dt_{0.0};
};

/** Measurement rollout factor with Jacobians propagated alongside the state.
 *
 * This is an alternative to TerminalAccelerationGyroMeasurementFactor. It has
 * the same residual and key layout, but predict() explicitly returns the
 * terminal-to-initial state Jacobian and one terminal-state Jacobian for every
 * control. State rows and columns are ordered [position, local rotation
 * tangent, velocity], and control columns are [world acceleration, body
 * angular rate]. Intermediate-state Jacobians are composed during propagation
 * and are not retained.
 */
class GTSAM_EXPORT TerminalStateFactor
    : public gtsam::NoiseModelFactor {
  public:
    using shared_ptr = boost::shared_ptr<TerminalStateFactor>;
    using StateJacobian = Eigen::Matrix<double, 9, 9>;
    using StateControlJacobian = Eigen::Matrix<double, 9, 6>;
    using gtsam::NoiseModelFactor::unwhitenedError;

    TerminalStateFactor() = default;
    TerminalStateFactor(
        Key pose_0, Key velocity_0, const gtsam::KeyVector& control_keys,
        const gtsam::Pose3& measured_pose_j, const gtsam::Vector3& measured_velocity_j,
        double dt, const SharedNoiseModel& model);

    gtsam::Vector unwhitenedError(const gtsam::Values& values,
                                  gtsam::OptionalMatrixVecType jacobians = nullptr) const override;
    DynamicsState predict(const DynamicsState& initial_state,
                          const std::vector<gtsam::Vector6>& controls) const;
    DynamicsState predict(const DynamicsState& initial_state,
                          const std::vector<gtsam::Vector6>& controls,
                          std::vector<StateControlJacobian>& control_jacobians) const;
    DynamicsState predict(const DynamicsState& initial_state,
                          const std::vector<gtsam::Vector6>& controls,
                          StateJacobian& initial_state_jacobian,
                          std::vector<StateControlJacobian>& control_jacobians) const;
    std::size_t horizon() const { return control_keys_.size(); }

  private:
    void propagate(DynamicsState& predicted, const gtsam::Vector6& control) const;
    // jacobian_x is the local one-step transition dx_(k+1)/dx_k, and
    // jacobian_control is the local one-step derivative dx_(k+1)/du_k.
    void propagate(DynamicsState& predicted, StateJacobian& jacobian_x,
                   StateControlJacobian& jacobian_control,
                   const gtsam::Vector6& control) const;
    gtsam::Vector9 residual(const DynamicsState& predicted,
                            gtsam::OptionalMatrixType predicted_jacobian = nullptr) const;

    gtsam::Key pose_0_, velocity_0_;
    gtsam::KeyVector control_keys_;
    gtsam::Pose3 measured_pose_j_;
    gtsam::Vector3 measured_velocity_j_{gtsam::Vector3::Zero()};
    double dt_{0.0};
};

class GTSAM_EXPORT DynamicFactor
    : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3,
                               gtsam::Vector3, gtsam::Vector3, gtsam::Vector4> {
  public:
    typedef boost::shared_ptr<DynamicFactor> shared_ptr;

    DynamicFactor() {}
    DynamicFactor(Key p_i, Key vel_i, Key omega_i, Key p_j, Key vel_j, Key omega_j, Key input_i,
                  float dt, float mass, gtsam::Vector3 inertia, gtsam::Vector3 rotor_pos,
                  gtsam::Vector3 drag_k, double ctt, double kmm, const SharedNoiseModel& model);

    virtual ~DynamicFactor() {}

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector3& omega_i, const gtsam::Pose3& pos_j,
                         const gtsam::Vector3& vel_j, const gtsam::Vector3& omega_j,
                         const gtsam::Vector4& input_i, gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone,
                         gtsam::OptionalMatrixType H5 = OptionalNone,
                         gtsam::OptionalMatrixType H6 = OptionalNone,
                         gtsam::OptionalMatrixType H7 = OptionalNone) const;

    gtsam::Vector6 Thrust_Torque(const gtsam::Vector4& rpm, const double& ct, const double& km,
                                 const gtsam::Vector3& rotor_pos, gtsam::Matrix64& Jac) const;
    gtsam::Vector6 Thrust_Torque(const gtsam::Vector4& rpm_square, const double& ct,
                                 const double& km, const gtsam::Vector3& rotor_pos,
                                 gtsam::Vector3& A) const;

  private:
    typedef DynamicFactor This;
    typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3,
                              gtsam::Vector3, gtsam::Vector3, gtsam::Vector4>
        Base;

    float dt_;
    float mass_;
    gtsam::Vector4 actuator_outputs_;

    gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity

    // !!! Check the order of PWM
    const gtsam::Matrix3 rk1_ = gtsam::Vector3(1, 1, 1).asDiagonal();
    const gtsam::Matrix3 rk2_ = gtsam::Vector3(1, -1, 1).asDiagonal();
    const gtsam::Matrix3 rk3_ = gtsam::Vector3(-1, -1, 1).asDiagonal();
    const gtsam::Matrix3 rk4_ = gtsam::Vector3(-1, 1, 1).asDiagonal();
    const gtsam::Vector3 axis = gtsam::Vector3(0, 0, 1);
    const gtsam::Matrix3 axis_mat = gtsam::skewSymmetric(axis);

    gtsam::Vector3 rot_inertia_, rotor_pos_, drag_k_;
    double ct, km;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/* position velocity rotation angular_velocity control_input */
class GTSAM_EXPORT DynamicsFactor
    : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                               gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  public:
    typedef boost::shared_ptr<DynamicsFactor> shared_ptr;

    DynamicsFactor() {}
    DynamicsFactor(Key p_i, Key vel_i, Key omega_i, Key input_i, Key p_j, Key vel_j, Key omega_j,
                   float dt, const SharedNoiseModel& model);

    virtual ~DynamicsFactor() {}

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector3& omega_i, const gtsam::Vector4& input_i,
                         const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j,
                         const gtsam::Vector3& omega_j, gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone,
                         gtsam::OptionalMatrixType H5 = OptionalNone,
                         gtsam::OptionalMatrixType H6 = OptionalNone,
                         gtsam::OptionalMatrixType H7 = OptionalNone) const;

  private:
    typedef DynamicsFactor This;
    typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                              gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
        Base;

    DynamicsParams dynamics_params_;

    float dt_;
};

/* position velocity rotation angular_velocity control_input:Force and Moment */
class GTSAM_EXPORT DynamicsFactorTm
    : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                               gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  public:
    typedef boost::shared_ptr<DynamicsFactorTm> shared_ptr;

    DynamicsFactorTm() {}
    DynamicsFactorTm(Key p_i, Key vel_i, Key omega_i, Key input_i, Key p_j, Key vel_j, Key omega_j,
                     float dt, const SharedNoiseModel& model);

    virtual ~DynamicsFactorTm() {}

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector3& omega_i, const gtsam::Vector4& input_i,
                         const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j,
                         const gtsam::Vector3& omega_j, gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone,
                         gtsam::OptionalMatrixType H5 = OptionalNone,
                         gtsam::OptionalMatrixType H6 = OptionalNone,
                         gtsam::OptionalMatrixType H7 = OptionalNone) const;

  private:
    typedef DynamicsFactorTm This;
    typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                              gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
        Base;

    DynamicsParams dynamics_params_;

    float dt_;
};

/*
 * MPC based FGO, generating Thrust and gyro
 */
class GTSAM_EXPORT DynamicsFactorTGyro
    : public NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3,
                               gtsam::Vector3> {
  public:
    typedef boost::shared_ptr<DynamicsFactorTGyro> shared_ptr;

    DynamicsFactorTGyro() {}
    DynamicsFactorTGyro(Key p_i, Key vel_i, Key input_i, Key p_j, Key vel_j, float dt, double mass_,
                        gtsam::Vector3 drag_k, const SharedNoiseModel& model);

    virtual ~DynamicsFactorTGyro() {}

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector4& input_i, const gtsam::Pose3& pos_j,
                         const gtsam::Vector3& vel_j, gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone,
                         gtsam::OptionalMatrixType H5 = OptionalNone) const;

  private:
    typedef DynamicsFactorTGyro This;
    typedef NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3,
                              gtsam::Vector3>
        Base;

    DynamicsParams dynamics_params_;

    gtsam::Vector3 drag_k_;

    double mass_;

    gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity

    float dt_;
};

/*
 * MPC based FGO, generating Thrust and gyro
 */
class GTSAM_EXPORT DynamicsFactorTGyroWall
    : public NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3,
                               gtsam::Vector3> {
  public:
    typedef boost::shared_ptr<DynamicsFactorTGyroWall> shared_ptr;

    DynamicsFactorTGyroWall() {}
    DynamicsFactorTGyroWall(Key p_i, Key vel_i, Key input_i, Key p_j, Key vel_j, float dt,
                            double mass_, gtsam::Vector3 drag_k, const SharedNoiseModel& model);

    virtual ~DynamicsFactorTGyroWall() {}

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector4& input_i, const gtsam::Pose3& pos_j,
                         const gtsam::Vector3& vel_j, gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone,
                         gtsam::OptionalMatrixType H5 = OptionalNone) const;

  private:
    typedef DynamicsFactorTGyroWall This;
    typedef NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3,
                              gtsam::Vector3>
        Base;

    DynamicsParams dynamics_params_;

    gtsam::Vector3 drag_k_;

    double mass_;

    gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity

    float dt_;
};

/*thrust's x and y components are nozero */
class GTSAM_EXPORT DynamicsFactorFullTM
    : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                               gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
  public:
    typedef boost::shared_ptr<DynamicsFactorFullTM> shared_ptr;

    DynamicsFactorFullTM() {}
    DynamicsFactorFullTM(Key p_i, Key vel_i, Key omega_i, Key tm_ij, Key p_j, Key vel_j,
                         Key omega_j, float dt, const SharedNoiseModel& model);

    virtual ~DynamicsFactorFullTM() {}

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector3& omega_i, const gtsam::Vector6& thrust_torque,
                         const gtsam::Pose3& pos_j, const gtsam::Vector3& vel_j,
                         const gtsam::Vector3& omega_j, gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone,
                         gtsam::OptionalMatrixType H5 = OptionalNone,
                         gtsam::OptionalMatrixType H6 = OptionalNone,
                         gtsam::OptionalMatrixType H7 = OptionalNone) const;

  private:
    typedef DynamicsFactorFullTM This;
    typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                              gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
        Base;

    DynamicsParams dynamics_params_;

    float dt_;
};

/* Force and Moments Between factor */
class GTSAM_EXPORT BetForceMoments : public NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> {
  public:
    typedef boost::shared_ptr<BetForceMoments> shared_ptr;

    BetForceMoments() {}
    BetForceMoments(Key input_i, Key input_j, const SharedNoiseModel& model);

    virtual ~BetForceMoments() {}

    Vector evaluateError(const gtsam::Vector4& input_i, const gtsam::Vector4& input_j,
                         gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone) const;

  private:
    typedef BetForceMoments This;
    typedef NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4> Base;
};

/* Force and Moments Between factor */
class GTSAM_EXPORT PointObsFactor : public NoiseModelFactor1<gtsam::Pose3> {
  public:
    typedef boost::shared_ptr<PointObsFactor> shared_ptr;

    PointObsFactor() {}
    PointObsFactor(Key p_i, gtsam::Vector3& obs, float safe_d, const SharedNoiseModel& model)
        : Base(model, p_i), obs_(obs), safe_d_(safe_d) {}

    virtual ~PointObsFactor() {}

    Vector evaluateError(const gtsam::Pose3& pi,
                         gtsam::OptionalMatrixType H1 = OptionalNone) const {
        Vector err;
        Matrix36 jac_t_posei;
        double d = (pi.translation(jac_t_posei) - obs_).norm();

        err = Vector1(1 / d - 1 / safe_d_ / 1.1);

        if (err(0) < 0) {
            err(0) = 0;
            if (H1) {
                *H1 = gtsam::Matrix16::Zero();
            }
        } else {
            // std::cout << " -- PointObs: [ " << err[0] << " ]" << std::endl;
            if (H1) {
                *H1 = -(pi.translation() - obs_).transpose() * jac_t_posei / d / d / d;
            }
        }

        return err;
    }

  private:
    typedef PointObsFactor This;
    typedef NoiseModelFactor1<gtsam::Pose3> Base;
    gtsam::Vector3 obs_;
    float safe_d_;
};

class GTSAM_EXPORT ControlLimitFactor : public NoiseModelFactor1<gtsam::Vector4> {
  public:
    typedef boost::shared_ptr<ControlLimitFactor> shared_ptr;

    ControlLimitFactor() {}
    ControlLimitFactor(Key input, const SharedNoiseModel& model, double low, double high,
                       double thr, double alpha)
        : Base(model, input), high_(high), low_(low), thr_(thr), alpha_(alpha){};

    virtual ~ControlLimitFactor() {}

    Vector evaluateError(const gtsam::Vector4& input,
                         gtsam::OptionalMatrixType H1 = OptionalNone) const;

  private:
    typedef ControlLimitFactor This;
    typedef NoiseModelFactor1<gtsam::Vector4> Base;

    double high_;
    double low_;
    double thr_;
    double alpha_;
};

class GTSAM_EXPORT ControlLimitTGyroFactor : public NoiseModelFactor1<gtsam::Vector4> {
  public:
    typedef boost::shared_ptr<ControlLimitTGyroFactor> shared_ptr;

    ControlLimitTGyroFactor() {}
    ControlLimitTGyroFactor(Key input, const SharedNoiseModel& model, double T_low, double T_high,
                            double Gyro_low, double Gyro_high, double T_thr, double Gyro_thr,
                            double alpha)
        : Base(model, input), T_high_(T_high), T_low_(T_low), Gyro_high_(Gyro_high),
          Gyro_low_(Gyro_low), T_thr_(T_thr), Gyro_thr_(Gyro_thr), alpha_(alpha){};

    virtual ~ControlLimitTGyroFactor() {}

    Vector evaluateError(const gtsam::Vector4& input,
                         gtsam::OptionalMatrixType H1 = OptionalNone) const;

  private:
    typedef ControlLimitTGyroFactor This;
    typedef NoiseModelFactor1<gtsam::Vector4> Base;

    double T_high_;
    double T_low_;

    double Gyro_high_;
    double Gyro_low_;

    double T_thr_;
    double Gyro_thr_;

    double alpha_;
};

} // namespace UAVFactor

#endif // __DYNAMICS_PLANNING_FACTOR_H__
