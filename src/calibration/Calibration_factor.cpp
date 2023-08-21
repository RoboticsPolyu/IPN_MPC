#include <calibration/Calibration_factor.h>

namespace UAVFactor
{
    DynamcisCaliFactorthrustMomentsNoG::DynamcisCaliFactorthrustMomentsNoG(Key p_i, Key vel_i, Key omega_i, Key tm_ij,
        Key p_j, Key vel_j, Key omega_j, Key im_key, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j, im_key)
        , dt_(dt)
        , mass_(mass) {};

    Vector DynamcisCaliFactorthrustMomentsNoG::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, 
        const gtsam::Vector3 &omega_i, const gtsam::Vector6 &thrust_torque,
        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
        const gtsam::Vector3 &inertia_moment,
        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
        boost::optional<Matrix &> H7, boost::optional<Matrix &> H8) const
    {
        gtsam::Vector12 err;
        return err;
    }


    DynamcisCaliFactorthrustMoments::DynamcisCaliFactorthrustMoments(Key p_i, Key vel_i, Key omega_i, Key tm_ij,
        Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j, im_key, rwg_key)
        , dt_(dt)
        , mass_(mass) {};

    Vector DynamcisCaliFactorthrustMoments::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i,
        const gtsam::Vector6 &thrust_torque, 
        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
        const gtsam::Vector3 &inertia_moment, const gtsam::Rot3 &rwg,
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

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;
        
        const Point3 p_w_mi = pos_i.translation(jac_t_posei);
        const Rot3   r_w_mi = pos_i.rotation(jac_r_posei);
        const Point3 p_w_mj = pos_j.translation(jac_t_posej);
        const Rot3   r_w_mj = pos_j.rotation(jac_r_posej);

        // position rotation velocity error
        gtsam::Matrix33 J_pe_rwg;
        gtsam::Vector3 p_err = p_w_mj - p_w_mi - vel_i * dt_ - 0.5f* (-rwg.rotate(gI_, J_pe_rwg) + r_w_mi.rotate(thrust_torque.head(3)/ mass_))* dt_* dt_;

        gtsam::Matrix33 J_rerr_rbj, J_rbi, J_gI;
        gtsam::Vector3 rot_err = Rot3::Logmap(r_w_mj.between(r_w_mi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));

        gtsam::Vector3 vel_err = vel_j - (vel_i + (- rwg.rotate(gI_, J_gI) + r_w_mi.rotate(thrust_torque.head(3) / mass_)) * dt_);

        double Ix = inertia_moment.x();
        double Iy = inertia_moment.y();
        double Iz = inertia_moment.z();

        // omage error
        gtsam::Matrix3 J, J_inv;
        J     << Ix, 0,  0,
                 0,  Iy, 0,
                 0,  0,  Iz;
        J_inv << 1.0f/Ix, 0,       0,
                 0,       1.0f/Iy, 0,
                 0,       0,       1.0f/ Iz;

        gtsam::Vector3 omega_err = omega_j - omega_i - J_inv * (thrust_torque.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;

        if (H1)
        {
            Matrix33 Jac_perr_p = - Matrix33::Identity();
            Matrix33 Jac_perr_r =   r_w_mi.matrix() * gtsam::skewSymmetric(thrust_torque.head(3)/mass_)* dt_* dt_ * 0.5f;
            Matrix33 Jac_rerr_r =   J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
            Matrix33 Jac_verr_r =   r_w_mi.matrix() * skewSymmetric(thrust_torque.head(3)/mass_) * dt_;

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
            Matrix33 Jac_perr_veli  = -Matrix33::Identity() * dt_;
            Matrix33 Jac_verr_v     = -Matrix33::Identity();
            J_e_v.block(0, 0, 3, 3) = Jac_perr_veli;
            J_e_v.block(6, 0, 3, 3) = Jac_verr_v;

            *H2 = J_e_v;
        }

        if (H3)
        {
            Matrix123 J_e_omage;
            J_e_omage.setZero();
            double a = 1.0f / inertia_moment.x() * (inertia_moment.z() - inertia_moment.y());
            double b = 1.0f / inertia_moment.y() * (inertia_moment.x() - inertia_moment.z());
            double c = 1.0f / inertia_moment.z() * (inertia_moment.y() - inertia_moment.x());
            
            Matrix3 d_omega;
            d_omega <<  0,              a * omega_i[2], a * omega_i[1],
                        b * omega_i[2], 0,              b * omega_i[0],
                        c * omega_i[1], c * omega_i[0], 0;

            Matrix33 Jac_r_omega        = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
            Matrix33 Jac_omega_omega    = - I_3x3 + d_omega * dt_;
            J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
            J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

            *H3 = J_e_omage;
        }

        if (H4)
        {
            Matrix126 J_e_thrust_moments;
            
            J_e_thrust_moments.setZero();
            J_e_thrust_moments.block(6, 0, 3, 3) = - r_w_mi.matrix() / mass_ * dt_;
            J_e_thrust_moments.block(9, 3, 3, 3) = - J_inv * dt_; 
            J_e_thrust_moments.block(0, 0, 3, 3) = - r_w_mi.matrix() / mass_* dt_* dt_* 0.5f;
            
            *H4 = J_e_thrust_moments;
        }

        if (H5)
        {
            Matrix126 J_e_posej;
            J_e_posej.setZero();
            J_e_posej.block(0, 0, 3, 6) = jac_t_posej;
            J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;
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
            Matrix123 J_e_iofm;
            J_e_iofm.setZero();
            double wx = omega_i.x(), wy = omega_i.y(), wz = omega_i.z();

            gtsam::Matrix3 Jac_jb, Jac_Jinv;
            Jac_jb  <<  -(Iz - Iy)/Ix/Ix* wy* wz,  - 1.0f/Ix* wy* wz,          1.0f/Ix* wy* wz,
                          1.0f/Iy* wx* wz,         -(Ix - Iz)/Iy/Iy* wx* wz, - 1.0f/Iy* wx* wz,
                        - 1.0f/Iz* wx* wy,           1.0f/Iz* wx* wy,         -(Iy - Ix)/Iz/Iz* wx* wy;

            double tqx = thrust_torque(3);
            double tqy = thrust_torque(4);
            double tqz = thrust_torque(5);
            // d I^-1 * (skew(w)* I* w) / d I = [w2*w3*(I3 - I2)/I1, w1*w3*(I1- I3)/I2, w1*w2*(I2 - I1)/I3]
            Jac_Jinv << 1.0/Ix/Ix * tqx , 0,              0, 
                        0,               1.0/Iy/Iy * tqy, 0, 
                        0,               0,               1.0/Iz/Iz * tqz;

            J_e_iofm.block(9, 0, 3, 3) = Jac_Jinv* dt_ + Jac_jb* dt_;

            *H8 = J_e_iofm;
        }

        if (H9)
        {
            Matrix123 J_e_rwg;
            J_e_rwg.setZero();
            J_e_rwg.block(0, 0, 3, 3) = J_pe_rwg * 0.5f * dt_ * dt_;
            J_e_rwg.block(6, 0, 3, 3) = dt_* J_gI;
            *H9 = J_e_rwg;
        }

        err.head(3)           = p_err;
        err.block(3, 0, 3, 1) = rot_err;
        err.block(6, 0, 3, 1) = vel_err;
        err.tail(3)           = omega_err;

        return err;
    };

    DynamcisCaliFactor_TM::DynamcisCaliFactor_TM(Key p_i, Key vel_i, Key omega_i, Key tm_ij,
        Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j, im_key, rwg_key)
        , dt_(dt)
        , mass_(mass) {};

    Vector DynamcisCaliFactor_TM::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i,
        const gtsam::Vector6 &thrust_torque, 
        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
        const gtsam::Vector3 &inertia_moment, const gtsam::Rot3 &rwg,
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

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;
        
        const Point3 p_w_mi = pos_i.translation(jac_t_posei);
        const Rot3   r_w_mi = pos_i.rotation(jac_r_posei);
        const Point3 p_w_mj = pos_j.translation(jac_t_posej);
        const Rot3   r_w_mj = pos_j.rotation(jac_r_posej);

        double dtt = dt_* dt_;
        double Ix = inertia_moment.x();
        double Iy = inertia_moment.y();
        double Iz = inertia_moment.z();

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
            Matrix123 J_e_omagej;
            J_e_omagej.setZero();
            J_e_omagej.block(9, 0, 3, 3) = J;
            *H7 = J_e_omagej;
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
        Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, Key p_key, Key kf_key, Key km_key, gtsam::Vector4 actuator_outputs, float dt, float mass, const SharedNoiseModel &model)
        : Base(model, p_i, vel_i, omega_i, p_j, vel_j, omega_j, im_key, rwg_key, p_key, kf_key, km_key)
        , dt_(dt)
        , mass_(mass)
        , actuator_outputs_(actuator_outputs) {};

    Vector DynamcisCaliFactor_RS::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
                             const gtsam::Vector3 &inertia_moment, const gtsam::Rot3 &rwg, 
                             const gtsam::Vector3 &rotor_pos, const double &ct, const double &km,
                             boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                             boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                             boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                             boost::optional<Matrix &> H7, boost::optional<Matrix &> H8,
                             boost::optional<Matrix &> H9, boost::optional<Matrix &> H10,
                             boost::optional<Matrix &> H11) const
    {
        // wTm_i = wTbi * bTm
        // wTm_j = wTbj * bTm

        gtsam::Vector12 err;

        /* Solving thrust and torque */
        // thrust      = ct * axis * actuator
        // moments     = ct * rotor_pos.cross(axis) +- km * axis;
        // thrust_sum  = ct * sum{axis_i * actuator_i}
        // moments_sum = ct * sum{rotor_pos_i.cross(axis_i)} +- sum{km * axis_i}

        gtsam::Vector3 axis      = gtsam::Vector3(0, 0, 1);
        gtsam::Vector3 back_axis = gtsam::Vector3(0, 0, 1);
        gtsam::Vector3 _thrust   = ct * axis;
        
        gtsam::Matrix3 rk1, rk2, rk3, rk4;
        gtsam::Matrix3 axis_mat = gtsam::skewSymmetric(axis);
        gtsam::Matrix3 back_axis_mat = gtsam::skewSymmetric(back_axis);

        // !!! Check the order of PWM
        rk1 = gtsam::Vector3( 1,  1, 1).asDiagonal();
        rk2 = gtsam::Vector3( 1, -1, 1).asDiagonal();
        rk3 = gtsam::Vector3(-1, -1, 1).asDiagonal();
        rk4 = gtsam::Vector3(-1,  1, 1).asDiagonal();

        gtsam::Vector3 _moment1 = ct * -axis_mat* rk1 * rotor_pos + ct * km * axis; // moments of 1rd, 2rd rotor
        gtsam::Vector3 _moment2 = ct * -axis_mat* rk2 * rotor_pos - ct * km * axis; // moments of 3rd, 4rd rotor
        gtsam::Vector3 _moment3 = ct * -axis_mat* rk3 * rotor_pos + ct * km * axis; // moments of 3rd, 4rd rotor
        gtsam::Vector3 _moment4 = ct * -axis_mat* rk4 * rotor_pos - ct * km * axis; // moments of 3rd, 4rd rotor

        gtsam::Matrix64 effectiveness_matrix; 
        effectiveness_matrix.setZero();
        gtsam::Vector4 real_thrust_4_rotors;

        for(uint i = 0; i < 4u; i++)
        {
            // double rel_pwm = actuators_(i) * 22.0f - 17000.0f; // pacasso
            // double rel_pwm = actuators_(i) * 19.0f - 12697.0f; // ampersand
            double rel_pwm = actuator_outputs_(i);
            real_thrust_4_rotors(i) = rel_pwm * rel_pwm;
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment4;

        gtsam::Vector6 thrust_torque = effectiveness_matrix * real_thrust_4_rotors;

        // std::cout << "effectiveness_matrix: " << effectiveness_matrix << std::endl;
        // std::cout << err << std::endl;
        // std::cout << "real_thrust_4_rotors" << real_thrust_4_rotors << "\n";
        // std::cout << "thrust_torque: " << thrust_torque << "\n";

        // gtsam::Matrix6 Jac_wTmi_wTbi, Jac_wTmi_bTm;
        // gtsam::Matrix6 Jac_wTmj_wTbj, Jac_wTmj_bTm;

        // gtsam::Pose3 wTmi = pos_i.compose(bTm, Jac_wTmi_wTbi, Jac_wTmi_bTm);
        // gtsam::Pose3 wTmj = pos_j.compose(bTm, Jac_wTmj_wTbj, Jac_wTmj_bTm);

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;
        
        const Point3 p_w_mi = pos_i.translation(jac_t_posei);
        const Rot3   r_w_mi = pos_i.rotation(jac_r_posei);
        const Point3 p_w_mj = pos_j.translation(jac_t_posej);
        const Rot3   r_w_mj = pos_j.rotation(jac_r_posej);

        double dtt = dt_* dt_;
        double Ix = inertia_moment.x();
        double Iy = inertia_moment.y();
        double Iz = inertia_moment.z();

        // omage error
        gtsam::Matrix3 J, J_inv;
        J     << Ix, 0, 0, 0, Iy, 0, 0, 0, Iz;
        J_inv << 1.0f/Ix, 0,       0,
                 0,       1.0f/Iy, 0,
                 0,       0,       1.0f/ Iz;
        gtsam::Matrix33 _unrbi_matrix = r_w_mi.inverse().matrix();
        
        // position rotation velocity error
        gtsam::Matrix33 J_pe_roti, J_rerr_rbj, J_rbi, J_ve_rot1, J_rwg;
        gtsam::Vector3  _gI_updated = rwg.rotate(gI_, J_rwg);

        gtsam::Vector3  pos_err = mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f * _gI_updated * dtt - p_w_mi, J_pe_roti) - 0.5f* thrust_torque.head(3)* dtt;
        gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mj.between(r_w_mi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
        gtsam::Vector3  vel_err = mass_ * r_w_mi.unrotate(vel_j - vel_i + _gI_updated * dt_, J_ve_rot1) - thrust_torque.head(3) * dt_;
        gtsam::Vector3  asp_err = J * (omega_j - omega_i) + skewSymmetric(omega_i)*J*omega_i*dt_ - thrust_torque.tail(3) * dt_;


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
            Matrix126 J_e_posej;
            J_e_posej.setZero();
            J_e_posej.block(0, 0, 3, 6) = mass_* _unrbi_matrix* jac_t_posej;
            J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;
            *H4 = J_e_posej; 
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
            Matrix123 J_e_omagej;
            J_e_omagej.setZero();
            J_e_omagej.block(9, 0, 3, 3) = J;
            *H6 = J_e_omagej;
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
        if(H9 || H10 || H11)
        {
            J_e_thrust_moments.setZero();
            J_e_thrust_moments.block(6, 0, 3, 3) = - I_3x3* dt_;
            J_e_thrust_moments.block(9, 3, 3, 3) = - I_3x3* dt_; 
            J_e_thrust_moments.block(0, 0, 3, 3) = - I_3x3* dtt* 0.5f;
        }

        if (H9)
        {
            gtsam::Matrix3  J_m1_rp  = ct * -axis_mat * rk1;
            gtsam::Matrix3  J_m2_rp  = ct * -axis_mat * rk2;
            gtsam::Matrix3  J_m3_rp  = ct * -axis_mat * rk3;
            gtsam::Matrix3  J_m4_rp  = ct * -axis_mat * rk4;

            // rotor position
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

            J_rotorpos = J_tm_rotor1 * real_thrust_4_rotors(0) + J_tm_rotor2 * real_thrust_4_rotors(1) 
                        + J_tm_rotor3 * real_thrust_4_rotors(2) + J_tm_rotor4 * real_thrust_4_rotors(3);
                    
            *H9 = J_e_thrust_moments * J_rotorpos;
        }

        if (H10)
        {
            gtsam::Matrix31 J_thrust_ct = axis;
            gtsam::Matrix31 J_m1_ct  = - axis_mat * rk1 * rotor_pos + km * axis;
            gtsam::Matrix31 J_m2_ct  = - axis_mat * rk2 * rotor_pos - km * axis;
            gtsam::Matrix31 J_m3_ct  = - axis_mat * rk3 * rotor_pos + km * axis;
            gtsam::Matrix31 J_m4_ct  = - axis_mat * rk4 * rotor_pos - km * axis;

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

            J_ct = J_tm_ct1 * real_thrust_4_rotors(0) + J_tm_ct2 * real_thrust_4_rotors(1) + 
                    J_tm_ct3 * real_thrust_4_rotors(2) + J_tm_ct4 * real_thrust_4_rotors(3);

            *H10 = J_e_thrust_moments * J_ct;
        }

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

            J_km = J_tm_km1 * real_thrust_4_rotors(0) + J_tm_km2 * real_thrust_4_rotors(1) 
                + J_tm_km3 * real_thrust_4_rotors(2) + J_tm_km4 * real_thrust_4_rotors(3);

            *H11 = J_e_thrust_moments * J_km;
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

        gtsam::Vector4 real_thrust_4_rotors;
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

            // double rel_pwm = actuators_(i) * 22.0f - 17000.0f;    //  (actuators_(i) - PWM_MIN_) / (PWM_MAX_ - PWM_MIN_); // pacasso
            // double rel_pwm = actuators_(i) * 19.0f - 12697.0f; // ampersand
            double rel_pwm = actuators_(i);
            
            real_thrust_4_rotors(i) = rel_pwm * rel_pwm;
            effectiveness_matrix.block(0, i, 3, 1) = _thrust;
        }

        effectiveness_matrix.block(3, 0, 3, 1) = _moment_1;
        effectiveness_matrix.block(3, 1, 3, 1) = _moment_2;
        effectiveness_matrix.block(3, 2, 3, 1) = _moment_3;
        effectiveness_matrix.block(3, 3, 3, 1) = _moment_4;

        err =  effectiveness_matrix * real_thrust_4_rotors - thrust_moments;
    // std::cout << "effectiveness_matrix: " << effectiveness_matrix << std::endl;
    // std::cout << err << std::endl;
    // std::cout << "real_thrust_4_rotors" << real_thrust_4_rotors << "\n";
    // std::cout << "thrust_moments: " << thrust_moments << "\n";
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

            J_rotorpos = J_tm_rotor1 * real_thrust_4_rotors(0) + J_tm_rotor2 * real_thrust_4_rotors(1) 
                       + J_tm_rotor3 * real_thrust_4_rotors(2) + J_tm_rotor4 * real_thrust_4_rotors(3);

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

            J_ct = J_tm_ct1 * real_thrust_4_rotors(0) + J_tm_ct2 * real_thrust_4_rotors(1) + 
                   J_tm_ct3 * real_thrust_4_rotors(2) + J_tm_ct4 * real_thrust_4_rotors(3);
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

            J_km = J_tm_km1 * real_thrust_4_rotors(0) + J_tm_km2 * real_thrust_4_rotors(1) 
                 + J_tm_km3 * real_thrust_4_rotors(2) + J_tm_km4 * real_thrust_4_rotors(3);
            *H4 = J_km;

        }

        return err;
    }

}