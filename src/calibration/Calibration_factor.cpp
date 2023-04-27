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
            Matrix123 J_e_inertia_moments;
            J_e_inertia_moments.setZero();
            double Ix = inertia_moments.x();
            double Iy = inertia_moments.y();
            double Iz = inertia_moments.z();

            gtsam::Matrix3 Jac_jb, Jac_Jinv;
            Jac_jb << - 1/Ix/Ix* omega_i.y()* omega_i.z(), - omega_i.y()* omega_i.z(),            omega_i.y()* omega_i.z(),
                        omega_i.x()* omega_i.z(),          - 1/Iy/Iy* omega_i.x()* omega_i.z(), - omega_i.x()* omega_i.z(),
                      - omega_i.x()* omega_i.y(),            omega_i.x()* omega_i.y(),          - 1/Iz/Iz* omega_i.x()* omega_i.y();
            
            Jac_Jinv << 1/Ix/Ix* trust_moments_ij(3) , 0, 0, 
                        0, 1/Iy/Iy* trust_moments_ij(4), 0, 
                        0, 0, 1/Iz/Iz* trust_moments_ij(5);
            J_e_inertia_moments.block(9, 0, 3, 3) = Jac_Jinv + Jac_jb;

            *H8 = J_e_inertia_moments;
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


    Vector AllocationCalibFactor::evaluateError(const gtsam::Vector3& trust, const gtsam::Vector3 & moments, 
        const gtsam::Vector3& rotor_pos, const gtsam::Rot3& rotor_axis, const double &ct, const double &km, const double &esc_factor,
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6, boost::optional<Matrix &> H7) const
    {
        gtsam::Vector6 err;

        if(H1)
        {

        }
        if(H2)
        {

        }
        
        if(H3)
        {

        }
        if(H4)
        {

        }
        if(H5)
        {

        }
        if(H6)
        {

        }
        if(H7)
        {

        }

        return err;
    }

    Vector AllocationCalibFactor2::evaluateError(const gtsam::Vector3& trust, const gtsam::Vector3 & moments, 
        const gtsam::Vector3& rotor_pos, const double &ct, const double &km, const double &esc_factor,
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3, 
        boost::optional<Matrix &> H4, boost::optional<Matrix &> H5, boost::optional<Matrix &> H6) const
    {
        gtsam::Vector6 err;
        gtsam::Vector6 trust_moments;
        trust_moments.head(3) = trust;
        trust_moments.tail(3) = moments;

        gtsam::Vector4 real_trust_4_rotors;
        gtsam::Matrix3 J_axis;

        // trust       = ct * axis * actuator
        // moments     = ct * rotor_pos.cross(axis) +- ct * km * axis;
        // trust_sum   = ct * sum{axis_i * actuator_i}
        // moments_sum = ct * sum{rotor_pos_i.cross(axis_i) +- ct * km * axis_i}

        gtsam::Vector3 axis       = gtsam::Vector3(0, 0, 1);
        gtsam::Vector3 _thrust    = ct * axis;
        
        gtsam::Matrix3 rk1, rk2, rk3, rk4;
        rk1 = gtsam::Vector3(1, -1, 1).asDiagonal();
        gtsam::Vector3 _moment_1 = rk1 * ct * rotor_pos.cross(axis) - ct * km * axis; // moments of 1rd, 2rd rotor
        
        rk2 = gtsam::Vector3(-1, 1, 1).asDiagonal();
        gtsam::Vector3 _moment_2 = rk2 * ct * rotor_pos.cross(axis) - ct * km * axis; // moments of 3rd, 4rd rotor

        rk3 = gtsam::Vector3(1, 1, 1).asDiagonal();
        gtsam::Vector3 _moment_3 = rk3 * ct * rotor_pos.cross(axis) + ct * km * axis; // moments of 3rd, 4rd rotor
        
        rk4 = gtsam::Vector3(-1, -1, 1).asDiagonal();
        gtsam::Vector3 _moment_4 = rk4 * ct * rotor_pos.cross(axis) + ct * km * axis; // moments of 3rd, 4rd rotor

        gtsam::Matrix31 J_trust_ct = axis;

        gtsam::Matrix3  J_m1_rp    = - rk1 * ct * gtsam::skewSymmetric(axis);
        gtsam::Matrix3  J_m2_rp    = - rk2 * ct * gtsam::skewSymmetric(axis);
        gtsam::Matrix3  J_m3_rp    = - rk3 * ct * gtsam::skewSymmetric(axis);
        gtsam::Matrix3  J_m4_rp    = - rk4 * ct * gtsam::skewSymmetric(axis);

        gtsam::Matrix31 J_m1_ct    =   rk1 * rotor_pos.cross(axis) - km * axis;
        gtsam::Matrix31 J_m2_ct    =   rk2 * rotor_pos.cross(axis) - km * axis;
        gtsam::Matrix31 J_m3_ct    =   rk3 * rotor_pos.cross(axis) + km * axis;
        gtsam::Matrix31 J_m4_ct    =   rk4 * rotor_pos.cross(axis) + km * axis;

        gtsam::Matrix31 J_m1_km    = - ct * axis;
        gtsam::Matrix31 J_m2_km    = - ct * axis;
        gtsam::Matrix31 J_m3_km    =   ct * axis;
        gtsam::Matrix31 J_m4_km    =   ct * axis;

        gtsam::Matrix64 effectiveness_matrix; 
        effectiveness_matrix.setZero();
        gtsam::Matrix41 J_trust_esc;

        for(uint i = 0; i < 4u; i++)
        {
            double rel_pwm = (actuators_(i) - PWM_MIN_) / (PWM_MAX_ - PWM_MIN_);
            real_trust_4_rotors(i) = esc_factor* rel_pwm * rel_pwm + (1 - esc_factor)* rel_pwm;
            J_trust_esc(i) = rel_pwm * rel_pwm - rel_pwm;
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment_1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment_2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment_3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment_4;

        err =  effectiveness_matrix * real_trust_4_rotors - trust_moments;

        if(H1)
        {
            gtsam::Matrix63 J_trust;
            J_trust.setZero();
            J_trust.block(0, 0, 3, 3) = gtsam::Matrix3::Identity();
            *H1 = J_trust;
        }
        if(H2)
        {
            gtsam::Matrix63 J_mb;
            J_mb.setZero();
            J_mb.block(3, 0, 3, 3) = gtsam::Matrix3::Identity();
            *H2 = J_mb;
        }
        
        if(H3)
        {
            gtsam::Matrix63 J_rotorpos;
            J_rotorpos.setZero();
            gtsam::Matrix63 J_tm_rotor1;
            J_tm_rotor1.setZero();
            J_tm_rotor1.block(3, 0, 3, 3) = J_m1_rp;
            gtsam::Matrix63 J_tm_rotor2;
            J_tm_rotor2.setZero();
            J_tm_rotor2.block(3, 0, 3, 3) = J_m2_rp;
            gtsam::Matrix63 J_tm_rotor3;
            J_tm_rotor3.setZero();
            J_tm_rotor3.block(3, 0, 3, 3) = J_m3_rp;
            gtsam::Matrix63 J_tm_rotor4;
            J_tm_rotor4.setZero();
            J_tm_rotor4.block(3, 0, 3, 3) = J_m4_rp;

            J_rotorpos = J_tm_rotor1 * real_trust_4_rotors(0) + J_tm_rotor2 * real_trust_4_rotors(1) + 
            J_tm_rotor3 * real_trust_4_rotors(2) + J_tm_rotor4 * real_trust_4_rotors(3);

            *H3 = J_rotorpos;
        }
        if(H4)
        {
            gtsam::Matrix61 J_ct;
            J_ct.setZero();
            gtsam::Matrix61 J_tm_ct1;
            J_tm_ct1.setZero();
            J_tm_ct1.block(0, 0, 3, 1) = J_trust_ct;
            J_tm_ct1.block(3, 0, 3, 1) = J_m1_ct;
            gtsam::Matrix61 J_tm_ct2;
            J_tm_ct2.setZero();
            J_tm_ct2.block(0, 0, 3, 1) = J_trust_ct;
            J_tm_ct2.block(3, 0, 3, 1) = J_m2_ct;
            gtsam::Matrix61 J_tm_ct3;
            J_tm_ct3.setZero();
            J_tm_ct3.block(0, 0, 3, 1) = J_trust_ct;
            J_tm_ct3.block(3, 0, 3, 1) = J_m3_ct;
            gtsam::Matrix61 J_tm_ct4;
            J_tm_ct4.setZero();
            J_tm_ct4.block(0, 0, 3, 1) = J_trust_ct;
            J_tm_ct4.block(3, 0, 3, 1) = J_m4_ct;

            J_ct = J_tm_ct1 * real_trust_4_rotors(0) + J_tm_ct2 * real_trust_4_rotors(1) + 
                                    J_tm_ct3 * real_trust_4_rotors(2) + J_tm_ct4 * real_trust_4_rotors(3);
            *H4 = J_ct;

        }
        if(H5)
        {
            gtsam::Matrix61 J_km;
            J_km.setZero();
            gtsam::Matrix61 J_tm_km1;
            J_tm_km1.setZero();
            J_tm_km1.block(3, 0, 3, 3) = J_m1_km;
            gtsam::Matrix61 J_tm_km2;
            J_tm_km2.setZero();
            J_tm_km2.block(3, 0, 3, 3) = J_m2_km;
            gtsam::Matrix61 J_tm_km3;
            J_tm_km3.setZero();
            J_tm_km3.block(3, 0, 3, 3) = J_m3_km;
            gtsam::Matrix61 J_tm_km4;
            J_tm_km4.setZero();
            J_tm_km4.block(3, 0, 3, 3) = J_m4_km;

            J_km = J_tm_km1 * real_trust_4_rotors(0) + J_tm_km2 * real_trust_4_rotors(1) + J_tm_km3 * real_trust_4_rotors(2) + J_tm_km4 * real_trust_4_rotors(3);
            *H5 = J_km;

        }
        if(H6)
        {
            gtsam::Matrix61 J_esc_factor;
            J_esc_factor.setZero();
            J_esc_factor = effectiveness_matrix* J_trust_esc;
            *H6 = J_esc_factor;
        }

        return err;
    }
}