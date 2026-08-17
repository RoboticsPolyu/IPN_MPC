#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/control/cbf_factor.h>

namespace UAVFactor {

// -------------------- VeCBFPdFactor --------------------
Vector VeCBFPdFactor::evaluateError(const gtsam::Pose3& pi, const gtsam::Vector3& vi,
                                    const gtsam::Vector4& ui, gtsam::OptionalMatrixType H1,
                                    gtsam::OptionalMatrixType H2,
                                    gtsam::OptionalMatrixType H3) const {
    Vector err;
    Matrix36 jac_ti;
    // err = (- Vector1(1.* safe_d_ * safe_d_)
    //      + (pi.translation(jac_t_posei) - obs_).transpose() * (pi.translation(jac_t_posei) -
    //      obs_));
    gtsam::Vector3 pi_t = pi.translation(jac_ti);
    gtsam::Vector3 p_p0 = pi_t - obs_;
    const double d = std::max(p_p0.norm(), cbf_detail::kMinimumDistance);
    gtsam::Vector1 hi =
        Vector1(1 / safe_d_ - 1 / d) + beta_ * p_p0.transpose() / d * (vi - obs_vel_);

    gtsam::Vector3 _e2(0., 0., 1.);
    gtsam::Vector3 _g(0., 0., 9.81);
    gtsam::Vector3 _ai = _e2 * ui(0);

    gtsam::Matrix36 J_ri;
    gtsam::Matrix3 J_r_ri;
    gtsam::Matrix3 J_r_ai;

    gtsam::Vector3 x_dot_0 = vi;
    gtsam::Vector3 x_dot_2 = -_g + pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
    err = hi + alpha_ * evaluateH_ti(pi_t, vi) * x_dot_0 +
          alpha_ * evaluateH_vi(pi_t) * x_dot_2; // h + alpha* dot h

    if (err(0) >= 0.0) {
        err(0) = 0.0;
        if (H1) {
            *H1 = gtsam::Matrix16::Zero();
        }

        if (H2) {
            *H2 = gtsam::Matrix13::Zero();
        }

        if (H3) {
            *H3 = gtsam::Matrix14::Zero();
        }
    } else {
        if (H1) {
            *H1 = evaluateH_ti_err(pi_t, pi.rotation(), vi, ui) * jac_ti +
                  alpha_ * evaluateH_vi(pi_t) * J_r_ri * J_ri;
        }

        if (H2) {
            *H2 = evaluateH_vi_err(pi_t, vi);
        }

        if (H3) {
            gtsam::Matrix14 h3;
            h3.setZero();
            gtsam::Matrix13 J_ai = alpha_ * evaluateH_vi(pi_t) * J_r_ai;
            h3.block<1, 1>(0, 0) = (J_ai * _e2);
            *H3 = h3;
        }
    }

    return err;
}


// -------------------- VeCBFPdFactorcCylinder --------------------
Vector VeCBFPdFactorcCylinder::evaluateError(const gtsam::Pose3& pi, const gtsam::Vector3& vi,
                                             const gtsam::Vector4& ui, gtsam::OptionalMatrixType H1,
                                             gtsam::OptionalMatrixType H2,
                                             gtsam::OptionalMatrixType H3) const {
    Vector err;
    Matrix36 jac_ti;

    // err = (- Vector1(1.* safe_d_ * safe_d_)
    //      + (pi.translation(jac_t_posei) - obs_).transpose() * (pi.translation(jac_t_posei) -
    //      obs_));
    gtsam::Vector3 pi_t = (_E12 * pi.translation(jac_ti));
    gtsam::Vector3 p_p0 = pi_t - obs_;

    gtsam::Vector3 _vi = (_E12 * vi);

    const double d = std::max(p_p0.norm(), cbf_detail::kMinimumDistance);
    gtsam::Vector1 hi =
        Vector1(1 / safe_d_ - 1 / d) + beta_ * p_p0.transpose() / d * (_vi - obs_vel_);

    gtsam::Vector3 _e2(0., 0., 1.);
    gtsam::Vector3 _g(0., 0., 9.81);
    gtsam::Vector3 _ai = _e2 * ui(0);

    gtsam::Matrix36 J_ri;
    gtsam::Matrix3 J_r_ri;
    gtsam::Matrix3 J_r_ai;

    gtsam::Vector3 x_dot_0 = _vi;
    gtsam::Vector3 x_dot_2 = -_g + pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
    err = hi + alpha_ * evaluateH_ti(pi_t, _vi) * x_dot_0 +
          alpha_ * evaluateH_vi(pi_t) * x_dot_2; // h + alpha* dot h

    if (err(0) >= 0.0) {
        err(0) = 0.0;
        if (H1) {
            *H1 = gtsam::Matrix16::Zero();
        }

        if (H2) {
            *H2 = gtsam::Matrix13::Zero();
        }

        if (H3) {
            *H3 = gtsam::Matrix14::Zero();
        }
    } else {
        if (H1) {
            *H1 = evaluateH_ti_err(pi_t, pi.rotation(), _vi, ui) * jac_ti +
                  alpha_ * evaluateH_vi(pi_t) * _E12 * J_r_ri * J_ri;
        }

        if (H2) {
            *H2 = evaluateH_vi_err(pi_t, _vi) * _E12;
        }

        if (H3) {
            gtsam::Matrix14 h3;
            h3.setZero();
            gtsam::Matrix13 J_ai = alpha_ * evaluateH_vi(pi_t) * J_r_ai;
            h3(0, 0) = (J_ai * _e2)(0);
            *H3 = h3;
        }
    }

    return err;
}

} // namespace UAVFactor
