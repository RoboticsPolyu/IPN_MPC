#pragma once

#include <ipn_mpc/control/dynamics_params.h>
#include <ipn_mpc/factors/gtsam_compatibility.h>

/** Energy consumed over one trajectory interval.
 *
 * The four control values are actuator commands. Their total power is modeled
 * with the same quadratic curve for every actuator:
 *
 *   P_actuator = sum_i (c0 + c1 u_i + c2 u_i^2).
 *
 * Aerodynamic drag follows the signed convention used by the dynamics
 * factors, F_drag_body = diag(drag_k) v_body. Consequently, the non-negative
 * power dissipated by drag (for non-positive drag coefficients) is
 *
 *   P_drag = -v_body' diag(drag_k) v_body,
 *   v_body = R_world_body' v_world.
 *
 * The one-dimensional residual is the interval energy
 *
 *   e = dt (P_actuator + P_drag).
 *
 * `model_params` stores [c0, c1, c2]. Units are determined by the actuator
 * command convention; when coefficients produce watts and dt is seconds, the
 * residual is joules. A one-dimensional noise model controls the energy-cost
 * weight in the factor graph.
 */
class GTSAM_EXPORT EnergyFactor
    : public NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, float> {
  public:
    using shared_ptr = boost::shared_ptr<EnergyFactor>;

    EnergyFactor() = default;
    EnergyFactor(Key p_i, Key vel_i, Key input_i, Key dt_i, gtsam::Vector3 drag_k,
                 gtsam::Vector3 model_params, const SharedNoiseModel& model);

    ~EnergyFactor() override = default;

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector4& input_i, double dt,
                         gtsam::OptionalMatrixType H1 = OptionalNone,
                         gtsam::OptionalMatrixType H2 = OptionalNone,
                         gtsam::OptionalMatrixType H3 = OptionalNone,
                         gtsam::OptionalMatrixType H4 = OptionalNone) const;

    Vector evaluateError(const gtsam::Pose3& pos_i, const gtsam::Vector3& vel_i,
                         const gtsam::Vector4& input_i, const float& dt,
                         gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
                         gtsam::OptionalMatrixType H3,
                         gtsam::OptionalMatrixType H4) const override;

  private:
    using This = EnergyFactor;
    using Base = NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, float>;

    gtsam::Vector3 drag_k_{gtsam::Vector3::Zero()};
    gtsam::Vector3 model_params_{gtsam::Vector3::Zero()};

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
