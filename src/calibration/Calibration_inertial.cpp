#include "calibration/Calibration_Inertial.h"

namespace UAVFactor
{

    gtsam::Vector InertialEgdeGS::evaluateError(const gtsam::Pose3 &pose_i, const gtsam::Vector3 &vel_i, const gtsam::Pose3 &pose_j, const gtsam::Vector3 &vel_j,
                                                 const Rot3 &rwg, const double &scale, const imuBias::ConstantBias &bias_i,
                                                 boost::optional<Matrix &> H_pi,
                                                 boost::optional<Matrix &> H_vi,
                                                 boost::optional<Matrix &> H_pj,
                                                 boost::optional<Matrix &> H_vj,
                                                 boost::optional<Matrix &> H_rwg,
                                                 boost::optional<Matrix &> H_scale,
                                                 boost::optional<Matrix &> H_b) const
    {
        gtsam::Vector9 imu_pim_error;
        gtsam::Vector3 e_r, e_v, e_p;
        gtsam::Matrix36 H1, H2, H3, H4;

        gtsam::Rot3 rot_i = pose_i.rotation(&H1);
        gtsam::Vector3 p_i = pose_i.translation(&H2);
        gtsam::Rot3 rot_j = pose_j.rotation(&H3);
        gtsam::Vector3 p_j = pose_j.translation(&H4);

        gtsam::Matrix96 D_biasCorrected_bias;
        gtsam::Vector9 biasCorrected = _PIM_.biasCorrectedDelta(bias_i, H_b ? &D_biasCorrected_bias : 0);

        gtsam::Matrix3 J_r_dr, J_exp_dr;
        e_r = gtsam::Rot3::Logmap(gtsam::Rot3::Expmap(dR(biasCorrected), J_exp_dr).inverse() * rot_i.inverse() * rot_j, J_r_dr);
        gtsam::Matrix3 J_er_xi_dr;
        J_er_xi_dr = -(J_r_dr * ((rot_i.inverse() * rot_j).inverse() * gtsam::Rot3::Expmap(dR(biasCorrected))).matrix()) * J_exp_dr;

        gtsam::Matrix3 J_ev_ri, J_ep_ri, J_ev_1, J_ep_2;
        e_v = rot_i.unrotate(scale * vel_j - scale * vel_i - rwg * gI_ * dt_, J_ev_ri, J_ev_1)                                 - dV(biasCorrected);
        e_p = rot_i.unrotate(scale * p_j - scale * p_i - scale * vel_i * dt_ - 0.5 * (rwg * gI_ * dt_ * dt_), J_ep_ri, J_ep_2) - dP(biasCorrected);
        imu_pim_error << e_r, e_v, e_p;

        gtsam::Matrix93 J_e_pi, J_e_pj, J_e_vi, J_e_vj, J_e_ri, J_e_rj;
        if (H_pi)
        {
            gtsam::Matrix96 J_e_Pi;
            J_e_pi.setZero();
            J_e_pi.block<3, 3>(6, 0) = J_ep_2 * -scale; // de/pi
            J_e_ri.setZero();
            J_e_ri.block<3, 3>(0, 0) = J_r_dr * -(rot_j.inverse() * rot_i).matrix(); // dr/ri
            J_e_ri.block<3, 3>(3, 0) = J_ev_ri; // dv/ri
            J_e_ri.block<3, 3>(6, 0) = J_ep_ri; // dp/ri
            J_e_Pi = J_e_pi * H2 + J_e_ri * H1; // de/Pi = de/pi* pi/Pi + de/ri* ri/Pi
            *H_pi = J_e_Pi;
        }

        if (H_pj)
        {
            gtsam::Matrix96 J_e_Pj;
            J_e_pj.setZero();
            J_e_pj.block<3, 3>(6, 0) = J_ep_2 * scale; //de/pj
            J_e_rj.setZero();
            J_e_rj.block<3, 3>(0, 0) = J_r_dr; // dr/rj(de/rj)
            J_e_Pj = J_e_pj * H4 + J_e_rj * H3; // de/Pj = de/pj* pj/Pj + de/rj* rj/pj
            *H_pj = J_e_Pj;
        }

        if (H_vi)
        {
            J_e_vi.setZero();
            J_e_vi.block<3, 3>(3, 0) = -J_ev_1 * scale; // dv/vi
            *H_vi = J_e_vi;
        }

        if (H_vj)
        {
            J_e_vj.setZero();
            J_e_vj.block<3, 3>(3, 0) = J_ev_1 * scale; // dv/vj
            *H_vj = J_e_vj;
        }

        if (H_scale)
        {
            gtsam::Matrix91 J_e_s;
            J_e_s.setZero();
            J_e_s.block<3, 1>(3, 0) = J_ev_1 * (vel_j - vel_i) * scale; // dv/dscale
            J_e_s.block<3, 1>(6, 0) = J_ep_2 * (p_j - p_i - vel_i * dt_) * scale; // dp/dscale
            *H_scale = J_e_s;
        }

        if (H_rwg)
        {
            gtsam::Matrix93 J_e_r;
            J_e_r.setZero();
            J_e_r.block<3, 3>(3, 0) = J_ev_1 * rwg.matrix() * gtsam::skewSymmetric(gI_) * dt_; 
            J_e_r.block<3, 3>(6, 0) = 0.5 * J_ep_2 * rwg.matrix() * gtsam::skewSymmetric(gI_) * dt_ * dt_;
            *H_rwg = J_e_r;
        }

        if (H_b)
        {
            gtsam::Matrix96 J_e_b;
            J_e_b.setZero();
            J_e_b.block<3, 6>(0, 0) = J_er_xi_dr * D_biasCorrected_bias.block<3, 6>(0, 0); // R
            J_e_b.block<3, 6>(6, 0) = -D_biasCorrected_bias.block<3, 6>(3, 0);             // P
            J_e_b.block<3, 6>(3, 0) = -D_biasCorrected_bias.block<3, 6>(6, 0);             // V
            *H_b = J_e_b;
        }
        return imu_pim_error;
    }

}; // namespace gtsam_optimizer
