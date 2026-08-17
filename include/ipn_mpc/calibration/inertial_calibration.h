#pragma once

#include <ipn_mpc/control/dynamics_params.h>
#include <ipn_mpc/factors/gtsam_compatibility.h>

using namespace gtsam_wrapper;

namespace UAVFactor {

class GTSAM_EXPORT InertialEgdeGS
    : public gtsam_wrapper::NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3,
                                              gtsam::Vector3, gtsam::Rot3, double,
                                              imuBias::ConstantBias> {
    typedef InertialEgdeGS This;
    typedef gtsam_wrapper::NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3,
                                             gtsam::Vector3, gtsam::Rot3, double,
                                             imuBias::ConstantBias>
        Base;

    InertialEgdeGS() {}

  public:
    InertialEgdeGS(gtsam::Key posei_key, gtsam::Key veli_key, gtsam::Key posej_key,
                   gtsam::Key velj_key, gtsam::Key rwg_key, gtsam::Key scale_key, gtsam::Key b_key,
                   const gtsam::PreintegratedImuMeasurements& p)
        : Base(noiseModel::Gaussian::Covariance(p.preintMeasCov()), posei_key, veli_key, posej_key,
               velj_key, rwg_key, scale_key, b_key),
          _PIM_(p), dt_(p.deltaTij()), gI_(p.params()->n_gravity) {}

    static Eigen::Block<Vector9, 3, 1> dR(Vector9& v) {
        return v.segment<3>(0);
    }
    static Eigen::Block<Vector9, 3, 1> dP(Vector9& v) {
        return v.segment<3>(3);
    }
    static Eigen::Block<Vector9, 3, 1> dV(Vector9& v) {
        return v.segment<3>(6);
    }
    static Eigen::Block<const Vector9, 3, 1> dR(const Vector9& v) {
        return v.segment<3>(0);
    }
    static Eigen::Block<const Vector9, 3, 1> dP(const Vector9& v) {
        return v.segment<3>(3);
    }
    static Eigen::Block<const Vector9, 3, 1> dV(const Vector9& v) {
        return v.segment<3>(6);
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const gtsam::Vector3& vel_i,
                                const gtsam::Pose3& pose_j, const gtsam::Vector3& vel_j,
                                const Rot3& rwg, const double& scale,
                                const imuBias::ConstantBias& bias_i, gtsam::OptionalMatrixType H_pi,
                                gtsam::OptionalMatrixType H_vi, gtsam::OptionalMatrixType H_pj,
                                gtsam::OptionalMatrixType H_vj, gtsam::OptionalMatrixType H_rwg,
                                gtsam::OptionalMatrixType H_scale,
                                gtsam::OptionalMatrixType H_b) const;

  private:
    gtsam::PreintegratedImuMeasurements _PIM_;
    double dt_;
    gtsam::Vector3 gI_;
};
}; // namespace UAVFactor
