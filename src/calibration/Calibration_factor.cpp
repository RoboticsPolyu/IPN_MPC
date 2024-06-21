#include <calibration/Calibration_factor.h>

namespace UAVFactor
{


    DynamcisCaliFactor_TM::DynamcisCaliFactor_TM(Key p_i, Key vel_i, Key omega_i, Key tm_ij,
        Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j, im_key, rwg_key)
        , dt_(dt)
        , mass_(mass) {};

    Vector DynamcisCaliFactor_TM::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i,
        const gtsam::Vector6 &thrust_torque, 
        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
        const gtsam::Vector3 &rot_inertia, const gtsam::Rot3 &rwg,
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
        boost::optional<Matrix &> H7, boost::optional<Matrix &> H8,
        boost::optional<Matrix &> H9) const
    {
        // wTm_i = wTbi * ITb
        // wTm_j = wTbj * ITb

        gtsam::Vector12 err;
        // gtsam::Matrix6 Jac_wTmi_wTbi, Jac_wTmi_bTm;
        // gtsam::Matrix6 Jac_wTmj_wTbj, Jac_wTmj_bTm;

        // gtsam::Pose3 wTmi = pos_i.compose(ITb, Jac_wTmi_wTbi, Jac_wTmi_bTm);
        // gtsam::Pose3 wTmj = pos_j.compose(ITb, Jac_wTmj_wTbj, Jac_wTmj_bTm);

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;
        
        const Point3 p_w_mi = pos_i.translation(jac_t_posei);
        const Rot3   r_w_mi = pos_i.rotation(jac_r_posei);
        const Point3 p_w_mj = pos_j.translation(jac_t_posej);
        const Rot3   r_w_mj = pos_j.rotation(jac_r_posej);

        double dtt = dt_* dt_;
        double Ix = rot_inertia.x();
        double Iy = rot_inertia.y();
        double Iz = rot_inertia.z();

        // omage error
        gtsam::Matrix3 J, J_inv;
        J     << Ix, 0,  0,
                 0,  Iy, 0,
                 0,  0,  Iz;
        J_inv << 1.0f/Ix, 0,       0,
                 0,       1.0f/Iy, 0,
                 0,       0,       1.0f/ Iz;
        gtsam::Matrix33 _unrbi_matrix = r_w_mi.inverse().matrix();

        // position rotation velocity error
        gtsam::Matrix33 J_pe_roti;
        gtsam::Vector3  p_err   = mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f* gI_* dtt - p_w_mi, J_pe_roti) - 0.5f* thrust_torque.head(3)* dtt;
        gtsam::Matrix33 J_rerr_rbj, J_rbi, J_gI;
        gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mj.between(r_w_mi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
        gtsam::Matrix33 J_ve_rot1;
        gtsam::Vector3  vel_err = mass_ * r_w_mi.unrotate(vel_j - vel_i + gI_* dt_, J_ve_rot1) - thrust_torque.head(3) * dt_;
        gtsam::Vector3 omega_err = J * (omega_j - omega_i) + skewSymmetric(omega_i)*J*omega_i*dt_ - thrust_torque.tail(3) * dt_;

        if (H1)
        {
            Matrix33 Jac_perr_p = - mass_* _unrbi_matrix;
            Matrix33 Jac_perr_r =   mass_* J_pe_roti;
            Matrix33 Jac_rerr_r =   J_rbi;
            Matrix33 Jac_verr_r =   mass_* J_ve_rot1;

            Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
            Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
            Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

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
            Matrix33 Jac_perr_veli  = - mass_* _unrbi_matrix * dt_;
            Matrix33 Jac_verr_v     = - mass_* _unrbi_matrix;
            J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
            J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

            *H2 = J_e_v;
        }

        if (H3)
        {
            Matrix123 J_e_omage;
            J_e_omage.setZero();
            double a = Iz - Iy;
            double b = Ix - Iz;
            double c = Iy - Ix;
            
            Matrix3 d_omega;
            d_omega <<  0,              a * omega_i[2], a * omega_i[1],
                        b * omega_i[2], 0,              b * omega_i[0],
                        c * omega_i[1], c * omega_i[0], 0;

            Matrix33 Jac_r_omega        = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
            Matrix33 Jac_omega_omega    = - J + d_omega* dt_;
            J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
            J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

            *H3 = J_e_omage;
        }

        if (H4)
        {
            Matrix126 J_e_thrust_moments;
            
            J_e_thrust_moments.setZero();
            J_e_thrust_moments.block(6, 0, 3, 3) = - I_3x3* dt_;
            J_e_thrust_moments.block(9, 3, 3, 3) = - I_3x3* dt_; 
            J_e_thrust_moments.block(0, 0, 3, 3) = - I_3x3* dtt* 0.5f;
            
            *H4 = J_e_thrust_moments;
        }

        if (H5)
        {
            Matrix126 J_e_posej;
            J_e_posej.setZero();
            J_e_posej.block(0, 0, 3, 6) = mass_* _unrbi_matrix* jac_t_posej;
            J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;
            *H5 = J_e_posej; 
        }

        if (H6)
        {
            Matrix123 J_e_vj;
            J_e_vj.setZero();
            J_e_vj.block(6, 0, 3, 3) = mass_* _unrbi_matrix;
            *H6 = J_e_vj;
        }

        if (H7)
        {
            Matrix123 J_e_omegaj;
            J_e_omegaj.setZero();
            J_e_omegaj.block(9, 0, 3, 3) = J;
            *H7 = J_e_omegaj;
        }

        if (H8)
        {
            Matrix123 J_e_iofm;
            J_e_iofm.setZero();
            double wx = omega_i.x(), wy = omega_i.y(), wz = omega_i.z();

            gtsam::Matrix3 Jac_jb;
            Jac_jb  <<    0,           -1.0f* wy* wz, 1.0f* wy* wz,
                          1.0f* wx* wz, 0,           -1.0f* wx* wz,
                        - 1.0f* wx* wy, 1.0f* wx* wy, 0;

            gtsam::Vector3 delta_omega = omega_j - omega_i;
            gtsam::Matrix3 _omega = delta_omega.asDiagonal();
            J_e_iofm.block(9, 0, 3, 3) = _omega + Jac_jb* dt_;

            *H8 = J_e_iofm;
        }

        if (H9)
        {
            Matrix123 J_e_rwg;
            J_e_rwg.setZero();
            J_e_rwg.block(0, 0, 3, 3) = mass_* _unrbi_matrix * 0.5f * dtt;
            J_e_rwg.block(6, 0, 3, 3) = mass_* _unrbi_matrix * dt_;
            *H9 = J_e_rwg;
        }

        err.head(3)           = p_err;
        err.block(3, 0, 3, 1) = rot_err;
        err.block(6, 0, 3, 1) = vel_err;
        err.tail(3)           = omega_err;

        return err;
    };



    DynamcisCaliFactor_RS::DynamcisCaliFactor_RS(Key p_i, Key vel_i, Key omega_i,
        Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, Key p_key, Key kf_key, Key km_key, Key bTm_key, Key drag_key,gtsam::Vector4 actuator_outputs, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, p_j, vel_j, omega_j, im_key, rwg_key, p_key, kf_key, km_key, bTm_key, drag_key)
        , dt_(dt)
        , mass_(mass)
        , actuator_outputs_(actuator_outputs) {};



    /* Solving thrust and torque */
    // thrust      = ct * axis * actuator
    // moments     = ct * rotor_pos.cross(axis) +- km * axis;
    // thrust_sum  = ct * sum{axis_i * actuator_i}
    // moments_sum = ct * sum{rotor_pos_i.cross(axis_i)} +- sum{km * axis_i}
    Vector DynamcisCaliFactor_RS::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
        const gtsam::Vector3 &rot_inertia, const gtsam::Rot3 &rwg, const gtsam::Vector3 &rotor_pos, const double &ct, const double &km, 
        const gtsam::Pose3& ITb, const gtsam::Vector3 &drag_k,
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3, 
        boost::optional<Matrix &> H4, boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
        boost::optional<Matrix &> H7, boost::optional<Matrix &> H8, boost::optional<Matrix &> H9, 
        boost::optional<Matrix &> H10, boost::optional<Matrix &> H11, boost::optional<Matrix &> H12, 
        boost::optional<Matrix &> H13) const
    {
        gtsam::Vector12 err;
        gtsam::Vector3 axis      = gtsam::Vector3(0, 0, 1);
        gtsam::Vector3 _thrust   = ct * axis;
        gtsam::Matrix3 axis_mat  = gtsam::skewSymmetric(axis);
        gtsam::Matrix3 rk1, rk2, rk3, rk4;
        // !!! Check the order of PWM
        rk1 = gtsam::Vector3( 1,  1, 1).asDiagonal(); 
        rk2 = gtsam::Vector3( 1, -1, 1).asDiagonal(); 
        rk3 = gtsam::Vector3(-1, -1, 1).asDiagonal();
        rk4 = gtsam::Vector3(-1,  1, 1).asDiagonal();

        gtsam::Vector3 rp1, rp2, rp3, rp4;
        rp1 = rk1 * rotor_pos;
        gtsam::Vector3 _moment1 = - ct * axis_mat * rp1 + ct * km * axis; // moments of 1rd rotor
        rp2 = rk2 * rotor_pos;
        gtsam::Vector3 _moment2 = - ct * axis_mat * rp2 - ct * km * axis; // moments of 2rd rotor
        rp3 = rk3 * rotor_pos;
        gtsam::Vector3 _moment3 = - ct * axis_mat * rp3 + ct * km * axis; // moments of 3rd rotor
        rp4 = rk4 * rotor_pos;
        gtsam::Vector3 _moment4 = - ct * axis_mat * rp4 - ct * km * axis; // moments of 4rd rotor

        gtsam::Matrix64 effectiveness_matrix; 
        effectiveness_matrix.setZero();
        gtsam::Vector4 rpm_square;

        for(uint i = 0; i < 4u; i++)
        {
            double rel_pwm = actuator_outputs_(i);
            rpm_square(i) = rel_pwm * rel_pwm;
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment4;

        gtsam::Vector6 thrust_torque = effectiveness_matrix * rpm_square;

        // std::cout << "effectiveness_matrix: " << effectiveness_matrix / ct << std::endl;
        // std::cout << "rpm_square" << rpm_square << "\n";
        // std::cout << "thrust_torque: " << thrust_torque << "\n";

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;

        gtsam::Matrix6 Jac_posei, Jac_i_bTm;
        gtsam::Matrix6 Jac_posej, Jac_j_bTm;

        gtsam::Pose3 w_T_mi = pos_i.compose(ITb, Jac_posei, Jac_i_bTm);
        gtsam::Pose3 w_T_mj = pos_j.compose(ITb, Jac_posej, Jac_j_bTm);

        const Point3 p_w_mi = w_T_mi.translation(jac_t_posei);
        const Rot3   r_w_mi = w_T_mi.rotation(jac_r_posei);
        const Point3 p_w_mj = w_T_mj.translation(jac_t_posej);
        const Rot3   r_w_mj = w_T_mj.rotation(jac_r_posej);

        double dtt = dt_* dt_;
        double Ix = rot_inertia.x();
        double Iy = rot_inertia.y();
        double Iz = rot_inertia.z();
        gtsam::Matrix3 J, J_inv;
        J     << Ix,      0, 0, 0, Iy,      0, 0, 0, Iz;
        J_inv << 1.0f/Ix, 0, 0, 0, 1.0f/Iy, 0, 0, 0, 1.0f/ Iz;
        gtsam::Matrix33 _unrbi_matrix = r_w_mi.inverse().matrix();
        
        // position rotation velocity angular_speed error
        gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
        // J_rerr_rbj, J_rbi;
        gtsam::Vector3  _gI_updated = rwg.rotate(gI_, J_rwg);
        gtsam::Matrix33 J_ri, J_rj, J_dr;
        gtsam::Vector3  pos_err = mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f * _gI_updated * dtt - p_w_mi, J_pe_roti) - 0.5f* thrust_torque.head(3)* dtt;
        gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - 0.5f * (omega_i + omega_j) * dt_; 
        // Rot3::Logmap(r_w_mj.between(r_w_mi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
        gtsam::Matrix3 drag_matrix;
        drag_matrix.setZero();
        drag_matrix.diagonal() << drag_k;
        gtsam::Vector3  vel_err = mass_ * r_w_mi.unrotate(vel_j - vel_i + _gI_updated * dt_, J_ve_rot1) - thrust_torque.head(3) * dt_ - mass_ * drag_matrix * r_w_mi.unrotate(vel_i, J_dv_rit, J_dv_v) * dt_;

        gtsam::Vector3  asp_err = J * (omega_j - omega_i) + skewSymmetric(omega_i)*J*omega_i*dt_ - thrust_torque.tail(3) * dt_;

        Matrix126 J_e_pi, J_e_posej;
        if (H1)
        {
            Matrix33 Jac_perr_p = - mass_* _unrbi_matrix;
            Matrix33 Jac_perr_r =   mass_* J_pe_roti;
            Matrix33 Jac_rerr_r =   J_dr * J_ri;
            Matrix33 Jac_verr_r =   mass_* J_ve_rot1 - mass_ * drag_matrix * J_dv_rit * dt_;

            Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
            Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
            Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

            J_e_pi.setZero();
            J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
            J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
            J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

            *H1 = J_e_pi* Jac_posei;
        }

        if (H2)
        {
            Matrix123 J_e_v;
            J_e_v.setZero();
            Matrix33 Jac_perr_veli  = - mass_* _unrbi_matrix * dt_;
            Matrix33 Jac_verr_v     = - mass_* _unrbi_matrix;
            J_e_v.block(0, 0, 3, 3) =   Jac_perr_veli;
            J_e_v.block(6, 0, 3, 3) =   Jac_verr_v - mass_ * drag_matrix * dt_ * J_dv_v;

            *H2 = J_e_v;
        }

        if (H3)
        {
            Matrix123 J_e_omage;
            J_e_omage.setZero();
            double a = Iz - Iy;
            double b = Ix - Iz;
            double c = Iy - Ix;
            
            Matrix3 d_omega;
            d_omega <<  0,              a * omega_i[2], a * omega_i[1],
                        b * omega_i[2], 0,              b * omega_i[0],
                        c * omega_i[1], c * omega_i[0], 0;

            Matrix33 Jac_r_omega        = - 0.5f * gtsam::I_3x3 * dt_; // SO3::ExpmapDerivative(omega_i * dt_) * dt_;
            Matrix33 Jac_omega_omega    = - J + d_omega* dt_;
            J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
            J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

            *H3 = J_e_omage;
        }

        if (H4)
        {
            J_e_posej.setZero();
            J_e_posej.block(0, 0, 3, 6) = mass_* _unrbi_matrix* jac_t_posej;
            J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
            *H4 = J_e_posej * Jac_posej; 
        }

        if (H5)
        {
            Matrix123 J_e_vj;
            J_e_vj.setZero();
            J_e_vj.block(6, 0, 3, 3) = mass_* _unrbi_matrix;
            *H5 = J_e_vj;
        }

        if (H6)
        {
            Matrix123 J_e_omegaj;
            J_e_omegaj.setZero();
            J_e_omegaj.block(3, 0, 3, 3) = - 0.5f * gtsam::I_3x3 * dt_;
            J_e_omegaj.block(9, 0, 3, 3) = J;
            *H6 = J_e_omegaj;
        }

        if (H7)
        {
            Matrix123 J_e_iofm;
            J_e_iofm.setZero();
            double wx = omega_i.x(), wy = omega_i.y(), wz = omega_i.z();

            gtsam::Matrix3 Jac_jb;
            Jac_jb  <<    0,           -1.0f* wy* wz, 1.0f* wy* wz,
                          1.0f* wx* wz, 0,           -1.0f* wx* wz,
                        - 1.0f* wx* wy, 1.0f* wx* wy, 0;

            gtsam::Vector3 delta_omega = omega_j - omega_i;
            gtsam::Matrix3 _omega = delta_omega.asDiagonal();
            J_e_iofm.block(9, 0, 3, 3) = _omega + Jac_jb* dt_;

            *H7 = J_e_iofm;
        }

        if (H8)
        {
            Matrix123 J_e_rwg;
            J_e_rwg.setZero();
            J_e_rwg.block(0, 0, 3, 3) = mass_* _unrbi_matrix * 0.5f * dtt * J_rwg;
            J_e_rwg.block(6, 0, 3, 3) = mass_* _unrbi_matrix * dt_ * J_rwg;
            *H8 = J_e_rwg;
        }

        Matrix126 J_e_thrust_moments;
        Matrix123 J_moments;

        if(H9 || H10 || H11)
        {
            J_e_thrust_moments.setZero();
            J_e_thrust_moments.block(6, 0, 3, 3) = - I_3x3 * dt_;
            J_e_thrust_moments.block(9, 3, 3, 3) = - I_3x3 * dt_; 
            J_e_thrust_moments.block(0, 0, 3, 3) = - I_3x3 * dtt* 0.5f;

            J_moments.setZero();
            J_moments.block(9, 0, 3, 3) = - I_3x3 * dt_;
        }

        // rotor position
        if (H9)
        {
            gtsam::Matrix3  J_m1_rp  = - ct * axis_mat * rk1;
            gtsam::Matrix3  J_m2_rp  = - ct * axis_mat * rk2;
            gtsam::Matrix3  J_m3_rp  = - ct * axis_mat * rk3;
            gtsam::Matrix3  J_m4_rp  = - ct * axis_mat * rk4;

            gtsam::Matrix3 J_thrust_rp;

            J_thrust_rp = J_m1_rp * rpm_square(0) + J_m2_rp * rpm_square(1) 
                        + J_m3_rp * rpm_square(2) + J_m4_rp * rpm_square(3);
            *H9 = J_moments * J_thrust_rp;
        }

        // k_f
        if (H10)
        {
            gtsam::Matrix31 J_thrust_ct = axis;
            gtsam::Matrix31 J_m1_ct  = (rk1 * rotor_pos).cross(axis) + km * axis;
            gtsam::Matrix31 J_m2_ct  = (rk2 * rotor_pos).cross(axis) - km * axis;
            gtsam::Matrix31 J_m3_ct  = (rk3 * rotor_pos).cross(axis) + km * axis;
            gtsam::Matrix31 J_m4_ct  = (rk4 * rotor_pos).cross(axis) - km * axis;

            // c_t
            gtsam::Matrix61 J_ct;
            J_ct.setZero();
            gtsam::Matrix61 J_tm_ct1;
            J_tm_ct1.setZero();
            J_tm_ct1.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct1.block(3, 0, 3, 1) = J_m1_ct;
            gtsam::Matrix61 J_tm_ct2;
            J_tm_ct2.setZero();
            J_tm_ct2.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct2.block(3, 0, 3, 1) = J_m2_ct;
            gtsam::Matrix61 J_tm_ct3;
            J_tm_ct3.setZero();
            J_tm_ct3.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct3.block(3, 0, 3, 1) = J_m3_ct;
            gtsam::Matrix61 J_tm_ct4;
            J_tm_ct4.setZero();
            J_tm_ct4.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct4.block(3, 0, 3, 1) = J_m4_ct;

            J_ct = J_tm_ct1 * rpm_square(0) + J_tm_ct2 * rpm_square(1) + 
                    J_tm_ct3 * rpm_square(2) + J_tm_ct4 * rpm_square(3);

            *H10 = J_e_thrust_moments * J_ct;
        }

        // k_m
        if (H11)
        {
            gtsam::Matrix31 J_m1_km  =   ct * axis;
            gtsam::Matrix31 J_m2_km  = - ct * axis;
            gtsam::Matrix31 J_m3_km  =   ct * axis;
            gtsam::Matrix31 J_m4_km  = - ct * axis;

            // km
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

            J_km = J_tm_km1 * rpm_square(0) + J_tm_km2 * rpm_square(1) 
                 + J_tm_km3 * rpm_square(2) + J_tm_km4 * rpm_square(3);

            *H11 = J_e_thrust_moments * J_km;
        }

        if (H12)
        {
            Matrix126 J_e_bTm;
            J_e_bTm.setZero();
            *H12 = J_e_pi * Jac_i_bTm + J_e_posej * Jac_j_bTm;
        }

        if (H13)
        {
            Matrix123 J_drag_k;
            J_drag_k.setZero();
            gtsam::Matrix3 _J_drag;
            _J_drag.setZero();
            _J_drag.diagonal() << r_w_mi.unrotate(vel_i);
            J_drag_k.block(6, 0, 3, 3) = - mass_ * _J_drag * dt_;

            *H13 = J_drag_k;
        }

        err.head(3)           = pos_err;
        err.block(3, 0, 3, 1) = rot_err;
        err.block(6, 0, 3, 1) = vel_err;
        err.tail(3)           = asp_err;

        return err;
    };

    DynamcisCaliFactor_RS_AB::DynamcisCaliFactor_RS_AB(Key p_i, Key vel_i, Key omega_i,
            Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, Key p_key, Key kf_key, Key km_key, Key bTm_key, Key drag_key, Key A_key, Key B_key, gtsam::Vector4 actuator_outputs, float dt, float mass, const SharedNoiseModel &model)
            : Base(model, p_i, vel_i, omega_i, p_j, vel_j, omega_j, im_key, rwg_key, p_key, kf_key, km_key, bTm_key, drag_key, A_key, B_key)
            , dt_(dt)
            , mass_(mass)
            , actuator_outputs_(actuator_outputs) {};


    gtsam::Vector6 DynamcisCaliFactor_RS_AB::Thrust_Torque(const gtsam::Vector4 & rpm_square, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos) const
    {   
        gtsam::Vector3 _thrust  = ct * axis;

        gtsam::Vector3 rp1, rp2, rp3, rp4;
        rp1 = rk1_ * rotor_pos;
        gtsam::Vector3 _moment1 = - ct * axis_mat * rp1 + ct * km * axis; // moments of 1rd rotor
        rp2 = rk2_ * rotor_pos;
        gtsam::Vector3 _moment2 = - ct * axis_mat * rp2 - ct * km * axis; // moments of 2rd rotor
        rp3 = rk3_ * rotor_pos;
        gtsam::Vector3 _moment3 = - ct * axis_mat * rp3 + ct * km * axis; // moments of 3rd rotor
        rp4 = rk4_ * rotor_pos;
        gtsam::Vector3 _moment4 = - ct * axis_mat * rp4 - ct * km * axis; // moments of 4rd rotor

        gtsam::Matrix64 effectiveness_matrix; 
        effectiveness_matrix.setZero();

        for(uint i = 0; i < 4u; i++)
        {
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment4;

        // std::cout << "effectiveness_matrix: \n" << effectiveness_matrix / ct << std::endl;
        gtsam::Vector6 thrust_torque = effectiveness_matrix * rpm_square;

        return thrust_torque;
    }

    gtsam::Vector6 DynamcisCaliFactor_RS_AB::Thrust_Torque(const gtsam::Vector4 & rpm_square, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos, gtsam::Vector3 & A) const
    {   
        gtsam::Vector3 _thrust  = ct * axis;

        gtsam::Vector3 rp1, rp2, rp3, rp4;
        rp1 = rk1_ * rotor_pos;
        gtsam::Vector3 _moment1 = - ct * axis_mat * rp1 + ct * km * axis; // moments of 1rd rotor
        rp2 = rk2_ * rotor_pos;
        gtsam::Vector3 _moment2 = - ct * axis_mat * rp2 - ct * km * axis; // moments of 2rd rotor
        rp3 = rk3_ * rotor_pos;
        gtsam::Vector3 _moment3 = - ct * axis_mat * rp3 + ct * km * axis; // moments of 3rd rotor
        rp4 = rk4_ * rotor_pos;
        gtsam::Vector3 _moment4 = - ct * axis_mat * rp4 - ct * km * axis; // moments of 4rd rotor

        gtsam::Matrix64 effectiveness_matrix; 
        effectiveness_matrix.setZero();

        for(uint i = 0; i < 4u; i++)
        {
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment4;

        // std::cout << "effectiveness_matrix: \n" << effectiveness_matrix / ct << std::endl;
        gtsam::Vector6 thrust_torque = effectiveness_matrix * rpm_square;

        gtsam::Matrix33 F_mat;
        F_mat.setZero();
        gtsam::Vector3 f_v(thrust_torque[2], thrust_torque[2], 0);
        F_mat.diagonal() << f_v;
        gtsam::Vector3 torque_bias_mass = F_mat* A;
        thrust_torque.tail(3) = thrust_torque.tail(3) - torque_bias_mass;
        return thrust_torque;
    }

    Vector DynamcisCaliFactor_RS_AB::evaluateError(
        const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, const gtsam::Vector3 &rot_inertia, const gtsam::Rot3 &rwg, 
        const gtsam::Vector3 &rotor_pos, const double &ct, const double &km, const gtsam::Pose3& ITb,
        const gtsam::Vector3 &drag_k, const gtsam::Vector3 &A, const gtsam::Vector3 &B, 
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
        boost::optional<Matrix &> H7, boost::optional<Matrix &> H8,
        boost::optional<Matrix &> H9, boost::optional<Matrix &> H10,
        boost::optional<Matrix &> H11, boost::optional<Matrix &> H12,
        boost::optional<Matrix &> H13, boost::optional<Matrix &> H14,
        boost::optional<Matrix &> H15) const
    {
        gtsam::Vector12 err;

        gtsam::Vector4 rpm_square;

        for(uint i = 0; i < 4u; i++)
        {
            double rel_pwm = actuator_outputs_(i);
            rpm_square(i) = rel_pwm * rel_pwm;
        }

        gtsam::Vector6 thrust_torque = Thrust_Torque(rpm_square, ct, km, rotor_pos);

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;

        gtsam::Matrix6 Jac_posei, Jac_i_bTm;
        gtsam::Matrix6 Jac_posej, Jac_j_bTm;

        gtsam::Pose3 w_T_mi = pos_i.compose(ITb, Jac_posei, Jac_i_bTm);
        gtsam::Pose3 w_T_mj = pos_j.compose(ITb, Jac_posej, Jac_j_bTm);

        const Point3 p_w_mi = w_T_mi.translation(jac_t_posei);
        const Rot3   r_w_mi = w_T_mi.rotation(jac_r_posei);
        const Point3 p_w_mj = w_T_mj.translation(jac_t_posej);
        const Rot3   r_w_mj = w_T_mj.rotation(jac_r_posej);

        double dtt = dt_* dt_;
        double Ix = rot_inertia.x();
        double Iy = rot_inertia.y();
        double Iz = rot_inertia.z();
        gtsam::Matrix3 J, J_inv;
        J     << Ix,      0, 0, 0, Iy,      0, 0, 0, Iz;
        J_inv << 1.0f/Ix, 0, 0, 0, 1.0f/Iy, 0, 0, 0, 1.0f/ Iz;
        gtsam::Matrix33 _unrbi_matrix = r_w_mi.inverse().matrix();
        
        gtsam::Matrix33 J_da_ri, J_da_v, A_mat, B_mat, F_mat;
        A_mat.setZero();
        B_mat.setZero();
        F_mat.setZero();
        A_mat.diagonal() << A;
        B_mat.diagonal() << B;
        gtsam::Vector3 f_v(thrust_torque[2], thrust_torque[2], 0);
        F_mat.diagonal() << f_v;

        // position rotation velocity angular_speed error
        gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
        // J_rerr_rbj, J_rbi;
        gtsam::Vector3  _gI_updated = rwg.rotate(gI_, J_rwg);
        gtsam::Matrix33 J_ri, J_rj, J_dr;
        
        gtsam::Vector3 b_v_i = vel_i + r_w_mi.rotate(gtsam::skewSymmetric(omega_i)* ITb.translation());
        gtsam::Vector3 b_v_j = vel_j + r_w_mj.rotate(gtsam::skewSymmetric(omega_j)* ITb.translation());

        double HVT = (b_v_i.x()* b_v_i.x() + b_v_i.y()* b_v_i.y());

        gtsam::Matrix33 J_obi_rbI, J_omega_i, J_obj_rbI, J_omega_j;

        gtsam::Vector3 b_omega_i = ITb.rotation().unrotate(omega_i, J_obi_rbI, J_omega_i);
        gtsam::Vector3 b_omega_j = ITb.rotation().unrotate(omega_j, J_obj_rbI, J_omega_j);

        gtsam::Vector3  pos_err = mass_ * r_w_mi.unrotate(p_w_mj - b_v_i * dt_ + 0.5f * _gI_updated * dtt - p_w_mi, J_pe_roti) - 0.5f* thrust_torque.head(3)* dtt;
        // gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - 0.5f * (omega_i + omega_j) * dt_; 
        // This would cause the heavy fluctuation of omega

        gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - b_omega_i * dt_; 

        gtsam::Matrix3 drag_matrix;
        drag_matrix.setZero();
        drag_matrix.diagonal() << drag_k;        

        gtsam::Vector3  vel_err = mass_ * r_w_mi.unrotate(b_v_j - b_v_i + _gI_updated * dt_, J_ve_rot1) - thrust_torque.head(3) * dt_ 
            - A.z() * HVT *  dt_ * gtsam::Vector3(0,0,1)
            - mass_ * drag_matrix * r_w_mi.unrotate(b_v_i, J_dv_rit, J_dv_v) * dt_; // - dT * dt_;

        // gtsam::Vector3  asp_err = J * (omega_j - omega_i) + skewSymmetric(omega_i)*J*omega_i*dt_ - thrust_torque.tail(3) * dt_ - A_mat * r_w_mi.unrotate(b_v_i, J_da_ri, J_da_v) * dt_ - B_mat * omega_i * dt_;

        gtsam::Vector3 torque_gyroscopic;
        float Ip = 0;
        float G3x =   actuator_outputs_[0] - actuator_outputs_[1] + actuator_outputs_[2] - actuator_outputs_[3];
        float G3y = - actuator_outputs_[0] + actuator_outputs_[1] - actuator_outputs_[2] + actuator_outputs_[3];

        gtsam::Matrix3 _torque_gyro_k;
        _torque_gyro_k << G3x, 0, 0, 0, G3y, 0, 0, 0, 0; // = (actuator_outputs_[0] - actuator_outputs_[1] + actuator_outputs_[2] - actuator_outputs_[3]) * axis_mat;

        torque_gyroscopic = A_mat * _torque_gyro_k * axis_mat * b_omega_i;

        gtsam::Vector3 torque_bias_mass = F_mat * A;

        gtsam::Vector3  asp_err = J * (b_omega_j - b_omega_i) + skewSymmetric(b_omega_i) * J * b_omega_i * dt_ 
            - thrust_torque.tail(3) * dt_ 
            + torque_bias_mass * dt_ 
            // + torque_gyroscopic * dt_ 
            - B_mat * omega_i * dt_;
        
        Matrix126 J_e_pi, J_e_posej;
        
        if (H1)
        {
            Matrix33 Jac_perr_p = - mass_* _unrbi_matrix;
            Matrix33 Jac_perr_r =   mass_* J_pe_roti;
            Matrix33 Jac_rerr_r =   J_dr * J_ri;
            Matrix33 Jac_verr_r =   mass_* J_ve_rot1 - mass_ * drag_matrix * J_dv_rit * dt_; // - A_mat * J_da_ri * dt_;

            Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
            Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
            Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;

            J_e_pi.setZero();
            J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
            J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
            J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

            *H1 = J_e_pi* Jac_posei;
        }

        if (H2)
        {
            Matrix123 J_e_v;
            J_e_v.setZero();
            Matrix33 Jac_perr_veli  = - mass_* _unrbi_matrix * dt_;
            Matrix33 Jac_verr_v     = - mass_* _unrbi_matrix;
            J_e_v.block(0, 0, 3, 3) =   Jac_perr_veli;
            J_e_v.block(6, 0, 3, 3) =   Jac_verr_v - mass_ * drag_matrix * dt_ * J_dv_v;

            *H2 = J_e_v;
        }

        if (H3)
        {
            Matrix123 J_e_omage;
            J_e_omage.setZero();
            double a = Iz - Iy;
            double b = Ix - Iz;
            double c = Iy - Ix;
            
            Matrix3 d_omega;
            d_omega <<  0,                a * b_omega_i[2], a * b_omega_i[1],
                        b * b_omega_i[2], 0,                b * b_omega_i[0],
                        c * b_omega_i[1], c * b_omega_i[0], 0;

            Matrix33 Jac_r_omega        = - gtsam::I_3x3 * dt_; // - 0.5f * gtsam::I_3x3 * dt_; // SO3::ExpmapDerivative(omega_i * dt_) * dt_;
            Matrix33 Jac_omega_omega    = - J + d_omega* dt_;
            J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
            J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega // - A_mat * _torque_gyro_k * dt_
                                        - B_mat * dt_;
            *H3 = J_e_omage* J_omega_i;
        }

        if (H4)
        {
            J_e_posej.setZero();
            J_e_posej.block(0, 0, 3, 6) = mass_* _unrbi_matrix* jac_t_posej;
            J_e_posej.block(3, 0, 3, 6) = J_dr * J_rj * jac_r_posej;
            *H4 = J_e_posej * Jac_posej; 
        }

        if (H5)
        {
            Matrix123 J_e_vj;
            J_e_vj.setZero();
            J_e_vj.block(6, 0, 3, 3) = mass_* _unrbi_matrix;
            *H5 = J_e_vj;
        }

        if (H6)
        {
            Matrix123 J_e_omegaj;
            J_e_omegaj.setZero();
            // J_e_omegaj.block(3, 0, 3, 3) = - 0.5f * gtsam::I_3x3 * dt_;
            J_e_omegaj.block(9, 0, 3, 3) = J;
            *H6 = J_e_omegaj* J_omega_j;
        }


        // inertia of moment
        if (H7)
        {
            Matrix123 J_e_iofm;
            J_e_iofm.setZero();
            double wx = b_omega_i.x(), wy = b_omega_i.y(), wz = b_omega_i.z();

            gtsam::Matrix3 Jac_jb;
            Jac_jb  <<    0,           -1.0f* wy* wz, 1.0f* wy* wz,
                          1.0f* wx* wz, 0,           -1.0f* wx* wz,
                        - 1.0f* wx* wy, 1.0f* wx* wy, 0;

            gtsam::Vector3 delta_omega = b_omega_j - b_omega_i;
            gtsam::Matrix3 _omega = delta_omega.asDiagonal();
            J_e_iofm.block(9, 0, 3, 3) = _omega + Jac_jb* dt_;

            *H7 = J_e_iofm;
        }

        // rwg
        if (H8)
        {
            Matrix123 J_e_rwg;
            J_e_rwg.setZero();
            J_e_rwg.block(0, 0, 3, 3) = mass_* _unrbi_matrix * 0.5f * dtt * J_rwg;
            J_e_rwg.block(6, 0, 3, 3) = mass_* _unrbi_matrix * dt_ * J_rwg;
            *H8 = J_e_rwg;
        }

        Matrix126 J_e_thrust_moments;
        Matrix123 J_moments;

        if(H9 || H10 || H11)
        {
            J_e_thrust_moments.setZero();
            J_e_thrust_moments.block(6, 0, 3, 3) = - I_3x3 * dt_; // - J_dT_T * dt_;
            J_e_thrust_moments.block(9, 3, 3, 3) = - I_3x3 * dt_; 
            J_e_thrust_moments.block(0, 0, 3, 3) = - I_3x3 * dtt* 0.5f;
            J_moments.setZero();
            J_moments.block(9, 0, 3, 3) = - I_3x3 * dt_;
        }

        // rotor position
        if (H9)
        {
            gtsam::Matrix3  J_m1_rp  = - ct * axis_mat * rk1_;
            gtsam::Matrix3  J_m2_rp  = - ct * axis_mat * rk2_;
            gtsam::Matrix3  J_m3_rp  = - ct * axis_mat * rk3_;
            gtsam::Matrix3  J_m4_rp  = - ct * axis_mat * rk4_;

            gtsam::Matrix3 J_thrust_rp;
            J_thrust_rp = J_m1_rp * rpm_square(0) + J_m2_rp * rpm_square(1) 
                        + J_m3_rp * rpm_square(2) + J_m4_rp * rpm_square(3);
            *H9 = J_moments * J_thrust_rp;
        }

        // k_f
        if (H10)
        {
            gtsam::Matrix31 J_thrust_ct = axis;
            gtsam::Matrix31 J_m1_ct  = (rk1_ * rotor_pos).cross(axis) + km * axis;
            gtsam::Matrix31 J_m2_ct  = (rk2_ * rotor_pos).cross(axis) - km * axis;
            gtsam::Matrix31 J_m3_ct  = (rk3_ * rotor_pos).cross(axis) + km * axis;
            gtsam::Matrix31 J_m4_ct  = (rk4_ * rotor_pos).cross(axis) - km * axis;

            // c_t
            gtsam::Matrix61 J_ct;
            J_ct.setZero();
            gtsam::Matrix61 J_tm_ct1;
            J_tm_ct1.setZero();
            J_tm_ct1.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct1.block(3, 0, 3, 1) = J_m1_ct;
            gtsam::Matrix61 J_tm_ct2;
            J_tm_ct2.setZero();
            J_tm_ct2.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct2.block(3, 0, 3, 1) = J_m2_ct;
            gtsam::Matrix61 J_tm_ct3;
            J_tm_ct3.setZero();
            J_tm_ct3.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct3.block(3, 0, 3, 1) = J_m3_ct;
            gtsam::Matrix61 J_tm_ct4;
            J_tm_ct4.setZero();
            J_tm_ct4.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct4.block(3, 0, 3, 1) = J_m4_ct;

            J_ct = J_tm_ct1 * rpm_square(0) + J_tm_ct2 * rpm_square(1) + 
                    J_tm_ct3 * rpm_square(2) + J_tm_ct4 * rpm_square(3);

            *H10 = J_e_thrust_moments * J_ct;
        }

        // k_m
        if (H11)
        {
            gtsam::Matrix31 J_m1_km  =   ct * axis;
            gtsam::Matrix31 J_m2_km  = - ct * axis;
            gtsam::Matrix31 J_m3_km  =   ct * axis;
            gtsam::Matrix31 J_m4_km  = - ct * axis;

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

            J_km = J_tm_km1 * rpm_square(0) + J_tm_km2 * rpm_square(1) 
                 + J_tm_km3 * rpm_square(2) + J_tm_km4 * rpm_square(3);

            *H11 = J_e_thrust_moments * J_km;
        }

        // Ibm
        if (H12)
        {
            Matrix126 J_e_bTm;
            J_e_bTm.setZero();
            *H12 = J_e_pi * Jac_i_bTm + J_e_posej * Jac_j_bTm;
        }

        // drag
        if (H13)
        {
            Matrix123 J_drag_k;
            J_drag_k.setZero();
            gtsam::Matrix3 _J_drag;
            _J_drag.setZero();
            _J_drag.diagonal() << r_w_mi.unrotate(b_v_i);
            J_drag_k.block(6, 0, 3, 3) = - mass_ * _J_drag * dt_;
            *H13 = J_drag_k;
        }

        // HoG and HVT Coeff
        if (H14)
        {
            Matrix123 J_e_A;
            J_e_A.setZero();
            gtsam::Matrix3 Jac_A;
            Jac_A.setZero();
            // Jac_A.diagonal() << _torque_gyro_k * axis_mat * b_omega_i * dt_;
            Jac_A = F_mat * dt_;
            J_e_A.block(9, 0, 3, 3) = Jac_A;
            J_e_A(8, 2) = - HVT *  dt_;
            *H14 = J_e_A;
        }

        // Viscous impact
        if (H15)
        {
            Matrix123 J_e_B;

            // - B_mat * omega_i * dt_
            J_e_B.setZero();
            gtsam::Matrix3 Jac_B;
            Jac_B.setZero();
            Jac_B.diagonal() << - b_omega_i * dt_;
            J_e_B.block(9, 0, 3, 3) = Jac_B;
            // J_e_B.block(6, 0, 3, 3) = - J_dT_B * dt_;
            *H15 = J_e_B;

        }


        err.head(3)           = pos_err;
        err.block(3, 0, 3, 1) = rot_err;
        err.block(6, 0, 3, 1) = vel_err;
        err.tail(3)           = asp_err;

        return err;
    };


    Vector AllocationCalibFactor::evaluateError(const gtsam::Vector3& thrust, const gtsam::Vector3 & moments, 
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


    Vector AllocationCalibFactor3::evaluateError(const gtsam::Vector6& thrust_moments, 
        const gtsam::Vector3& rotor_pos, const double &ct, const double &km,
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4) const
    {
        gtsam::Vector6 err;

        gtsam::Vector4 rpm_square;
        gtsam::Matrix3 J_axis;

        // thrust      = ct * axis * actuator
        // moments     = ct * rotor_pos.cross(axis) +- ct * km * axis;
        // thrust_sum  = ct * sum{axis_i * actuator_i}
        // moments_sum = ct * sum{rotor_pos_i.cross(axis_i) +- ct * km * axis_i}

        gtsam::Vector3 axis      = gtsam::Vector3(0, 0, 1);
        gtsam::Vector3 back_axis = gtsam::Vector3(0, 0, 1);
        gtsam::Vector3 _thrust   = ct * axis;
        
        gtsam::Matrix3 rk1, rk2, rk3, rk4;
        gtsam::Matrix3 axis_mat = gtsam::skewSymmetric(axis);
        gtsam::Matrix3 back_axis_mat = gtsam::skewSymmetric(back_axis);

        // !!! Check the order of PWM
        rk1 = gtsam::Vector3(1, -1, 1).asDiagonal();
        gtsam::Vector3 _moment_1 = ct * -axis_mat* rk1 * rotor_pos + km * axis; // moments of 1rd, 2rd rotor
        
        rk2 = gtsam::Vector3(-1, -1, 1).asDiagonal();
        gtsam::Vector3 _moment_2 = ct * -back_axis_mat* rk2 * rotor_pos - km * back_axis; // moments of 3rd, 4rd rotor

        
        rk3 = gtsam::Vector3(-1, 1, 1).asDiagonal();
        gtsam::Vector3 _moment_3 = ct * -back_axis_mat* rk3 * rotor_pos + km * back_axis; // moments of 3rd, 4rd rotor
        
        rk4 = gtsam::Vector3(1, 1, 1).asDiagonal();
        gtsam::Vector3 _moment_4 = ct * -axis_mat* rk4 * rotor_pos - km * axis; // moments of 3rd, 4rd rotor

        gtsam::Matrix31 J_thrust_ct = axis;

        gtsam::Matrix3  J_m1_rp  = ct * -axis_mat * rk1;
        gtsam::Matrix3  J_m2_rp  = ct * -back_axis_mat * rk2;
        gtsam::Matrix3  J_m3_rp  = ct * -back_axis_mat * rk3;
        gtsam::Matrix3  J_m4_rp  = ct * -axis_mat * rk4;

        gtsam::Matrix31 J_m1_ct  = - axis_mat * rk1 * rotor_pos;
        gtsam::Matrix31 J_m2_ct  = - back_axis_mat * rk2 * rotor_pos;
        gtsam::Matrix31 J_m3_ct  = - back_axis_mat * rk3 * rotor_pos;
        gtsam::Matrix31 J_m4_ct  = - axis_mat * rk4 * rotor_pos;

        gtsam::Matrix31 J_m1_km  =   axis;
        gtsam::Matrix31 J_m2_km  = - back_axis;
        gtsam::Matrix31 J_m3_km  =   back_axis;
        gtsam::Matrix31 J_m4_km  = - axis;

        gtsam::Matrix64 effectiveness_matrix; 
        effectiveness_matrix.setZero();
        gtsam::Matrix41 J_thrust_esc;

        for(uint i = 0; i < 4u; i++)
        {
            double rel_pwm = actuators_(i);    
            rpm_square(i) = rel_pwm * rel_pwm;
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment_1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment_2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment_3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment_4;

        err =  effectiveness_matrix * rpm_square - thrust_moments;

        // thrust + torque
        if(H1)
        {
            gtsam::Matrix6 J_t_m;
            J_t_m = - gtsam::Matrix6::Identity();
            *H1 = J_t_m;
        }
        
        // rotor position
        if(H2)
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

            J_rotorpos = J_tm_rotor1 * rpm_square(0) + J_tm_rotor2 * rpm_square(1) 
                       + J_tm_rotor3 * rpm_square(2) + J_tm_rotor4 * rpm_square(3);

            *H2 = J_rotorpos;
        }

        // c_t
        if(H3)
        {
            gtsam::Matrix61 J_ct;
            J_ct.setZero();
            gtsam::Matrix61 J_tm_ct1;
            J_tm_ct1.setZero();
            J_tm_ct1.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct1.block(3, 0, 3, 1) = J_m1_ct;
            gtsam::Matrix61 J_tm_ct2;
            J_tm_ct2.setZero();
            J_tm_ct2.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct2.block(3, 0, 3, 1) = J_m2_ct;
            gtsam::Matrix61 J_tm_ct3;
            J_tm_ct3.setZero();
            J_tm_ct3.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct3.block(3, 0, 3, 1) = J_m3_ct;
            gtsam::Matrix61 J_tm_ct4;
            J_tm_ct4.setZero();
            J_tm_ct4.block(0, 0, 3, 1) = J_thrust_ct;
            J_tm_ct4.block(3, 0, 3, 1) = J_m4_ct;

            J_ct = J_tm_ct1 * rpm_square(0) + J_tm_ct2 * rpm_square(1) + 
                   J_tm_ct3 * rpm_square(2) + J_tm_ct4 * rpm_square(3);
            *H3 = J_ct;

        }

        // km
        if(H4)
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

            J_km = J_tm_km1 * rpm_square(0) + J_tm_km2 * rpm_square(1) 
                 + J_tm_km3 * rpm_square(2) + J_tm_km4 * rpm_square(3);
            *H4 = J_km;

        }

        return err;
    }


    Vector RPM_TC_Factor::evaluateError(const gtsam::Vector4& rpm1, const gtsam::Vector4& rpm2, const double &k, 
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3) const
    {
        gtsam::Vector4 err;

        err = rpm2 - rpm1 - k * pwm_;

        if(H1)
        {
            *H1 = - gtsam::Matrix4::Identity();
        }

        if(H2)
        {
            *H2 = gtsam::Matrix4::Identity();
        }

        if(H3)
        {
            *H3 = - pwm_;
        }

        return err;
    }

    Vector MotorCalibFactor::evaluateError(const gtsam::Vector5 & params, boost::optional<Matrix &> H1) const
    {
        gtsam::Vector1 err;
        
        double v_a = battery_voltage_ * std::sqrt(pwm_ - params[3]) + pwm_ - params[4];
        
        // double v_a = std::sqrt(kk);
        // double v_a = kk * kk;

        gtsam::Vector3 dot3(v_a, - rotor_speed1_, - rotor_speed1_* rotor_speed1_);

        err = gtsam::Vector1(rotor_speed2_ - rotor_speed1_ - params.head(3).dot(dot3)* dt_);

        if(H1)
        {
            gtsam::Vector5 jac;
            jac.head(3) = - dot3 * dt_;
            jac[3] = - params[0] * (battery_voltage_ /2/std::sqrt(pwm_ - params[3])* -1)* dt_;
            jac[4] = - params[0] * - 1 * dt_;
            *H1 = jac.transpose();
        }
        return err;
    }

}