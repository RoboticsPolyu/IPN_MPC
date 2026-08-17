#pragma once

#include <ipn_mpc/control/dynamics_factor.h>
#include <ipn_mpc/control/dynamics_params.h>
#include <ipn_mpc/factors/gtsam_compatibility.h>

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

  private:
    using This = EnergyFactor;
    using Base = NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, float>;

    gtsam::Vector3 drag_k_;
    gtsam::Vector3 model_params_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
