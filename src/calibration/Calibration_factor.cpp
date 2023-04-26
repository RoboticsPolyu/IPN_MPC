#include <calibration/Calibration_factor.h>

namespace UAVFactor
{
   DynamcisCaliFactorTrustMoments::DynamcisCaliFactorTrustMoments(Key p_i, Key vel_i, Key omega_i, Key tm_ij,
                                  Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j, im_key, rwg_key)
        , dt_(dt)
        , mass_(mass) {};

   Vector DynamcisCaliFactorTrustMoments::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
                                        const gtsam::Vector6 &trust_moments_ij,
                                        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
                                        const gtsam::Vector3 &inertia_moments, const gtsam::Rot3 &rwg,
                                        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                                        boost::optional<Matrix &> H7, boost::optional<Matrix &> H8,
                                        boost::optional<Matrix &> H9) const
   {
        // wTm_i = wTbi * bTm
        // wTm_j = wTbj * bTm

        gtsam::Vector12 err;
        // gtsam::Matrix6 Jac_wTmi_wTbi, Jac_wTmi_bTm;
        // gtsam::Matrix6 Jac_wTmj_wTbj, Jac_wTmj_bTm;

        // gtsam::Pose3 wTmi = pos_i.compose(bTm, Jac_wTmi_wTbi, Jac_wTmi_bTm);
        // gtsam::Pose3 wTmj = pos_j.compose(bTm, Jac_wTmj_wTbj, Jac_wTmj_bTm);

        Matrix36 jac_t_wTmi, jac_t_wTmj;
        Matrix36 jac_r_wTmi, jac_r_wTmj;
        
        const Point3 p_w_mi = pos_i.translation(jac_t_wTmi);
        const Rot3 r_w_mi = pos_i.rotation(jac_r_wTmi);
        const Point3 p_w_mj = pos_j.translation(jac_t_wTmj);
        const Rot3 r_w_mj = pos_j.rotation(jac_r_wTmj);

        // position rotation velocity error
        gtsam::Vector3 p_err = p_w_mj - (p_w_mi + vel_i * dt_);
        gtsam::Matrix33 J_rerr_rbj, J_rbi, J_gI;
        gtsam::Vector3 rot_err = Rot3::Logmap(r_w_mj.between(r_w_mi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
        gtsam::Vector3 vel_err = vel_j - (vel_i + (- rwg.rotate(gI_, J_gI) + r_w_mi.rotate(trust_moments_ij.head(3) / mass_)) * dt_);

        // omage error
        gtsam::Matrix3 J, J_inv;
        J << inertia_moments.x(), 0, 0,
            0, inertia_moments.y(), 0,
            0, 0, inertia_moments.z();
        J_inv << 1.0 / inertia_moments.x(), 0, 0,
            0, 1.0 / inertia_moments.y(), 0,
            0, 0, 1.0 / inertia_moments.z();
        gtsam::Vector3 omega_err = omega_j - omega_i - J_inv * (trust_moments_ij.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;

        if (H1)
        {
            Matrix33 Jac_perr_p = -Matrix33::Identity();
            Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
            Matrix33 Jac_verr_r = r_w_mi.matrix() * skewSymmetric(trust_moments_ij.head(3) / mass_) * dt_;

            Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_wTmi;
            Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_wTmi;
            Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_wTmi;

            Matrix126 J_e_pi;
            J_e_pi.setZero();
            J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
            J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
            J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

            *H1 = J_e_pi;
        }

        if (H2)
        {
            Matrix123 J_e_v;
            J_e_v.setZero();
            Matrix33 Jac_perr_veli = -Matrix33::Identity() * dt_;
            Matrix33 Jac_verr_v = -Matrix33::Identity();
            J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
            J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

            *H2 = J_e_v;
        }

        if (H3)
        {
            Matrix123 J_e_omage;
            J_e_omage.setZero();
            double a = -1.0 / inertia_moments.x() * (inertia_moments.z() - inertia_moments.y());
            double b = -1.0 / inertia_moments.y() * (inertia_moments.x() - inertia_moments.z());
            double c = -1.0 / inertia_moments.z() * (inertia_moments.y() - inertia_moments.x());
            Matrix3 d_omega;
            d_omega << 0, a * omega_i[2], a * omega_i[1],
                b * omega_i[2], 0, b * omega_i[0],
                c * omega_i[1], c * omega_i[0], 0;

            Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
            Matrix33 Jac_omega_omega = -d_omega * dt_;
            J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
            J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

            *H3 = J_e_omage;
        }

        if (H4)
        {
            Matrix126 J_e_trust_moments;
            
            J_e_trust_moments.setZero();
            J_e_trust_moments.block(6, 0, 3, 3) = - r_w_mi.matrix() / mass_ * dt_;
            J_e_trust_moments.block(9, 3, 3, 3) = - J_inv * dt_;

            *H4 = J_e_trust_moments;
        }

        if (H5)
        {
            Matrix126 J_e_posej;
            J_e_posej.setZero();
            J_e_posej.block(0, 0, 3, 6) = jac_t_wTmj;
            J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_wTmj;

            *H5 = J_e_posej;
        }

        if (H6)
        {
            Matrix123 J_e_vj;
            J_e_vj.setZero();
            J_e_vj.block(6, 0, 3, 3) = Matrix33::Identity();
            *H6 = J_e_vj;
        }

        if (H7)
        {
            Matrix123 J_e_omagej;
            J_e_omagej.setZero();
            J_e_omagej.block(9, 0, 3, 3) = Matrix33::Identity();
            *H7 = J_e_omagej;
        }

        if (H8)
        {
            Matrix123 J_e_inert_moments;
            J_e_inert_moments.setZero();
            double Ix = inertia_moments.x();
            double Iy = inertia_moments.y();
            double Iz = inertia_moments.z();
            gtsam::Matrix3 Jac_jb, Jac_Jinv;
            Jac_jb << 1/Ix/Ix* omega_i.y()* omega_i.z(), - omega_i.y()* omega_i.z(), omega_i.y()* omega_i.z(),
                      omega_i.x()* omega_i.z(), 1/Iy/Iy* omega_i.y()* omega_i.z(), - omega_i.y()* omega_i.z(),
                      - omega_i.y()* omega_i.y(), omega_i.y()* omega_i.y(), 1/Iz/Iz* omega_i.y()* omega_i.y();
            
            Jac_Jinv << 1/Ix/Ix* trust_moments_ij(3) , 0, 0, 0, 1/Iy/Iy* trust_moments_ij(4), 0, 0, 0, 1/Iz/Iz* trust_moments_ij(5);
            J_e_inert_moments.block(9, 0, 3, 3) = Jac_Jinv + Jac_jb;
            *H8 = J_e_inert_moments;
        }

        if (H9)
        {
            Matrix123 J_e_rwg;
            J_e_rwg.setZero();
            J_e_rwg.block(6, 0, 3, 3) = dt_* J_gI;
            *H9 = J_e_rwg;
        }

        err.head(3) = p_err;
        err.block(3, 0, 3, 1) = rot_err;
        err.block(6, 0, 3, 1) = vel_err;
        err.tail(3) = omega_err;

        return err;
   };
}