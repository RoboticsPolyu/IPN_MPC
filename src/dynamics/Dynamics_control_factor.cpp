#include "quadrotor_simulator/Dynamics_control_factor.h"

namespace UAVFactor
{
   DynamicsFactor::DynamicsFactor(Key p_i, Key vel_i, Key omega_i, Key input_i,
                                  Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j),
         dt_(dt){};
   
   DynamicFactor::DynamicFactor(Key p_i, Key vel_i, Key omega_i, Key p_j, Key vel_j, Key omega_j, Key input_i,
            float dt, float mass, gtsam::Vector3 inertia, gtsam::Vector3 rotor_pos, gtsam::Vector3 drag_k, 
            double ctt, double kmm, const SharedNoiseModel &model)
            : Base(model, p_i, vel_i, omega_i, p_j, vel_j, omega_j, input_i)
            , dt_(dt)
            , mass_(mass)
            , rot_inertia_(inertia)
            , rotor_pos_(rotor_pos)
            , drag_k_(drag_k)
            , ct(ctt)
            , km(kmm){};


   gtsam::Vector6 DynamicFactor::Thrust_Torque(const gtsam::Vector4 & rpm, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos, gtsam::Matrix64 & Jac) const
   {   
         gtsam::Vector4 rpm_square;

         for(uint i = 0; i < 4u; i++)
         {
               double rel_pwm = rpm(i);
               rpm_square(i) = rel_pwm * rel_pwm;
         }

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

         gtsam::Matrix44 _J;
         _J.setZero();
         _J.diagonal() << rpm;

         Jac = effectiveness_matrix* 2 * _J;

         return thrust_torque;
   }

   gtsam::Vector6 DynamicFactor::Thrust_Torque(const gtsam::Vector4 & rpm_square, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos, gtsam::Vector3 & A) const
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

   Vector DynamicFactor::evaluateError(
         const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
         const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
         const gtsam::Vector4& input_i,
         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
         boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
         boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
         boost::optional<Matrix &> H7) const
   {
         gtsam::Vector12 err;
         gtsam::Matrix64 J_tt_rpm;

         gtsam::Vector6 thrust_torque = Thrust_Torque(input_i, ct, km, rotor_pos_, J_tt_rpm);
         
         Matrix36 jac_t_posei, jac_t_posej;
         Matrix36 jac_r_posei, jac_r_posej;

         const Point3 p_w_mi = pos_i.translation(jac_t_posei);
         const Rot3   r_w_mi = pos_i.rotation(jac_r_posei);
         const Point3 p_w_mj = pos_j.translation(jac_t_posej);
         const Rot3   r_w_mj = pos_j.rotation(jac_r_posej);

         double dtt = dt_* dt_;
         double Ix = rot_inertia_.x();
         double Iy = rot_inertia_.y();
         double Iz = rot_inertia_.z();
         gtsam::Matrix3 J, J_inv;
         J     << Ix,      0, 0, 0, Iy,      0, 0, 0, Iz;
         J_inv << 1.0f/Ix, 0, 0, 0, 1.0f/Iy, 0, 0, 0, 1.0f/ Iz;
         gtsam::Matrix33 _unrbi_matrix = r_w_mi.inverse().matrix();
         
         // position rotation velocity angular_speed error
         gtsam::Matrix33 J_rwg, J_pe_roti, J_ve_rot1, J_dv_rit, J_dv_v;
         // J_rerr_rbj, J_rbi;
         gtsam::Matrix33 J_ri, J_rj, J_dr;
         

         gtsam::Vector3  pos_err = mass_ * r_w_mi.unrotate(p_w_mj - vel_i * dt_ + 0.5f * gI_ * dtt - p_w_mi, J_pe_roti) - 0.5f* thrust_torque.head(3)* dtt;
         // gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - 0.5f * (omega_i + omega_j) * dt_; 
         // This would cause the heavy fluctuation of omega

         gtsam::Vector3  rot_err = Rot3::Logmap(r_w_mi.between(r_w_mj, J_ri, J_rj), J_dr) - omega_i * dt_; 

         gtsam::Matrix3 drag_matrix;
         drag_matrix.setZero();
         drag_matrix.diagonal() << drag_k_;        

         gtsam::Vector3  vel_err = mass_ * r_w_mi.unrotate(vel_j - vel_i + gI_ * dt_, J_ve_rot1) - thrust_torque.head(3) * dt_ - 
            mass_ * drag_matrix * r_w_mi.unrotate(vel_i, J_dv_rit, J_dv_v) * dt_; // - dT * dt_;

         gtsam::Vector3  asp_err = 
         J * (omega_j - omega_i) + skewSymmetric(omega_i) * J * omega_i * dt_ - thrust_torque.tail(3) * dt_;
         
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

               *H1 = J_e_pi;
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

               Matrix33 Jac_r_omega        = - gtsam::I_3x3 * dt_; // - 0.5f * gtsam::I_3x3 * dt_; // SO3::ExpmapDerivative(omega_i * dt_) * dt_;
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
               Matrix123 J_e_omegaj;
               J_e_omegaj.setZero();
               // J_e_omegaj.block(3, 0, 3, 3) = - 0.5f * gtsam::I_3x3 * dt_;
               J_e_omegaj.block(9, 0, 3, 3) = J;
               *H6 = J_e_omegaj;
         }

         Matrix126 J_e_thrust_moments;
         Matrix123 J_moments;

         J_e_thrust_moments.setZero();
         J_e_thrust_moments.block(6, 0, 3, 3) = - I_3x3 * dt_; // - J_dT_T * dt_;
         J_e_thrust_moments.block(9, 3, 3, 3) = - I_3x3 * dt_; 

         J_e_thrust_moments.block(0, 0, 3, 3) = - I_3x3 * dtt* 0.5f;

         J_moments.setZero();
         J_moments.block(9, 0, 3, 3) = - I_3x3 * dt_;

         if (H7)
         {
            *H7 = J_e_thrust_moments* J_tt_rpm;
         }

         err.head(3)           = pos_err;
         err.block(3, 0, 3, 1) = rot_err;
         err.block(6, 0, 3, 1) = vel_err;
         err.tail(3)           = asp_err;

         return err;
   };

   Vector DynamicsFactor::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, 
                                        const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
                                        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                                        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                                        boost::optional<Matrix &> H7) const
   {
      gtsam::Vector12 err;
      Matrix36 jac_t_posei, jac_t_posej;
      Matrix36 jac_r_posei, jac_r_posej;

      const Point3 p_w_i = pos_i.translation(jac_t_posei);
      const Rot3 r_w_bi  = pos_i.rotation(jac_r_posei);
      const Point3 p_w_j = pos_j.translation(jac_t_posej);
      const Rot3 r_w_bj  = pos_j.rotation(jac_r_posej);

      // force and moment
      Matrix4 K1, K2;
      K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
          0, 0, dynamics_params_.arm_length * dynamics_params_.k_f, -dynamics_params_.arm_length * dynamics_params_.k_f,
          -dynamics_params_.arm_length * dynamics_params_.k_f, dynamics_params_.arm_length * dynamics_params_.k_f, 0, 0,
          dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m, -dynamics_params_.k_m;

      K2 << 2.0 * input_i[0], 0, 0, 0,
            0, 2.0 * input_i[1], 0, 0,
            0, 0, 2.0 * input_i[2], 0,
            0, 0, 0, 2.0 * input_i[3];

      gtsam::Vector4 input2(input_i[0] * input_i[0], input_i[1] * input_i[1], input_i[2] * input_i[2], input_i[3] * input_i[3]);
      gtsam::Vector4 T_mb = K1 * input2;

      // position rotation velocity error
      gtsam::Vector3 p_err = p_w_j - (p_w_i + vel_i * dt_);
      // std::cout << "p_1:" << p_w_i << std::endl;
      // std::cout << "p_2:" << p_w_j << std::endl;
      // std::cout << "v_1:" << vel_i << std::endl;
      // std::cout << "p_error:" << p_err << std::endl;

      gtsam::Matrix33 J_rerr_rbj, J_rbi;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
      gtsam::Vector3 vel_err = vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))) * dt_);
      
      // std::cout << "v_error:" << vel_err << std::endl;
      // std::cout << "a_sum:" << -gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)) << std::endl;

      // std::cout << "Dynmaics Factor p_err: \n" << p_err << std::endl;
      // std::cout << "Dynmaics Factor r_err: \n" << rot_err << std::endl;
      // std::cout << "Dynmaics Factor v_err: \n" << vel_err << std::endl;

      // omage error
      gtsam::Matrix3 J, J_inv;
      J << dynamics_params_.Ixx, 0, 0,
          0, dynamics_params_.Iyy, 0,
          0, 0, dynamics_params_.Izz;
      J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
          0, 1.0 / dynamics_params_.Iyy, 0,
          0, 0, 1.0 / dynamics_params_.Izz;
      gtsam::Vector3 omega_err = omega_j - omega_i - J_inv * (T_mb.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;
      // std::cout << "Dynmaics Factor omage_err: \n" << omega_err << std::endl;

      if (H1)
      {
         Matrix33 Jac_perr_p = -Matrix33::Identity();
         Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
         Matrix33 Jac_verr_r = r_w_bi.matrix() * skewSymmetric(gtsam::Vector3(0, 0, T_mb[0] / dynamics_params_.mass)) * dt_;

         Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei;
         Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
         Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;
         // std::cout << Jac_perr_posei << std::endl;
         // std::cout << Jac_rerr_posei << std::endl;
         // std::cout << Jac_verr_posei << std::endl;

         Matrix126 J_e_pi;
         J_e_pi.setZero();
         J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
         J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
         J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

         *H1 = J_e_pi;

         // std::cout << "*H1: \n" << *H1 << std::endl;
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
         // std::cout << "*H2: \n" << *H2 << std::endl;
      }
      if (H3)
      {
         Matrix123 J_e_omage;
         J_e_omage.setZero();
         double a = 1.0f / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
         double b = 1.0f / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
         double c = 1.0f / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);
         Matrix3 d_omega;
         d_omega << -1,              a * omega_i[2], a * omega_i[1],
                    b * omega_i[2], -1,              b * omega_i[0],
                    c * omega_i[1], c * omega_i[0], -1;

         Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
         Matrix33 Jac_omega_omega = d_omega * dt_;
         J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
         J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

         *H3 = J_e_omage;
         // std::cout << "*H3: \n" << *H3 << std::endl;
      }
      if (H4)
      {
         Matrix124 J_e_input;
         J_e_input.setZero();
         Matrix124 _B;
         _B.setZero();
         _B.block(6, 0, 3, 1) = r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0f / dynamics_params_.mass) * dt_;
         _B.block(9, 1, 3, 3) = J_inv * dt_;

         J_e_input = -_B * K1 * K2;
         *H4 = J_e_input;
         // std::cout << "*H4: \n" << *H4 << std::endl;
      }
      if (H5)
      {
         Matrix126 J_e_posej;
         J_e_posej.setZero();
         J_e_posej.block(0, 0, 3, 6) = jac_t_posej;
         J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;

         *H5 = J_e_posej;
         // std::cout << "*H5: \n" << *H5 << std::endl;
      }
      if (H6)
      {
         Matrix123 J_e_vj;
         J_e_vj.setZero();
         J_e_vj.block(6, 0, 3, 3) = Matrix33::Identity();
         *H6 = J_e_vj;
         // std::cout << "*H6: \n" << *H6 << std::endl;
      }
      if (H7)
      {
         Matrix123 J_e_omagej;

         J_e_omagej.setZero();
         J_e_omagej.block(9, 0, 3, 3) = Matrix33::Identity();
         *H7 = J_e_omagej;
         // std::cout << "*H7: \n" << *H7 << std::endl;
      }

      err.head(3) = p_err;
      err.block(3, 0, 3, 1) = rot_err;
      err.block(6, 0, 3, 1) = vel_err;
      err.tail(3) = omega_err;
      // std::cout << "factor error: " << err.transpose() << std::endl;
      return err;
   };


   DynamicsFactorTm::DynamicsFactorTm(Key p_i, Key vel_i, Key omega_i, Key input_i,
                                  Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j),
         dt_(dt){};

   Vector DynamicsFactorTm::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, 
                                          const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
                                          const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                                          boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                          boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                          boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                                          boost::optional<Matrix &> H7) const
   {
      gtsam::Vector12 err;
      Matrix36 jac_t_posei, jac_t_posej;
      Matrix36 jac_r_posei, jac_r_posej;

      const Point3 p_w_i  = pos_i.translation(jac_t_posei);
      const Rot3   r_w_bi = pos_i.rotation(jac_r_posei);
      const Point3 p_w_j  = pos_j.translation(jac_t_posej);
      const Rot3   r_w_bj = pos_j.rotation(jac_r_posej);

      // force and moment

      // position rotation velocity error
      gtsam::Vector3 p_err = p_w_j - p_w_i - vel_i * dt_ - 
         0.5* (-gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)))*
         dt_* dt_;
      // std::cout << "p_1:" << p_w_i << std::endl;
      // std::cout << "p_2:" << p_w_j << std::endl;
      // std::cout << "v_1:" << vel_i << std::endl;
      // std::cout << "p_error:" << p_err << std::endl;

      gtsam::Matrix33 J_rerr_rbj, J_rbi;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
      gtsam::Vector3 vel_err = vel_j - 
         (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))) * dt_);
      
      // std::cout << "v_error:" << vel_err << std::endl;
      // std::cout << "a_sum:" << -gtsam::Vector3(0, 0, dynamics_params_.g) + 
      //    r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)) << std::endl;

      // // std::cout << "Dynmaics Factor p_err: \n" << p_err << std::endl;
      // std::cout << "Dynmaics Factor r_err: \n" << rot_err << std::endl;
      // std::cout << "Dynmaics Factor v_err: \n" << vel_err << std::endl;

      // omage error
      gtsam::Matrix3 J, J_inv;
      J << dynamics_params_.Ixx, 0,                    0,
           0,                    dynamics_params_.Iyy, 0,
           0,                    0,                    dynamics_params_.Izz;
      J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
          0, 1.0 / dynamics_params_.Iyy, 0,
          0, 0, 1.0 / dynamics_params_.Izz;
      gtsam::Vector3 omega_err = omega_j - omega_i - J_inv * (input_i.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;
      // std::cout << "Dynmaics Factor omage_err: \n" << omega_err << std::endl;

      if (H1)
      {
         Matrix33 Jac_perr_p = - Matrix33::Identity();
         Matrix33 Jac_perr_r = r_w_bi.matrix()* gtsam::skewSymmetric(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))
            * dt_* dt_ * 0.5f;
         Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
         Matrix33 Jac_verr_r = r_w_bi.matrix() * skewSymmetric(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)) * dt_;

         Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei + Jac_perr_r * jac_r_posei;
         Matrix36 Jac_rerr_posei = Jac_rerr_r * jac_r_posei;
         Matrix36 Jac_verr_posei = Jac_verr_r * jac_r_posei;
         // std::cout << Jac_perr_posei << std::endl;
         // std::cout << Jac_rerr_posei << std::endl;
         // std::cout << Jac_verr_posei << std::endl;

         Matrix126 J_e_pi;
         J_e_pi.setZero();
         J_e_pi.block(0, 0, 3, 6) = Jac_perr_posei;
         J_e_pi.block(3, 0, 3, 6) = Jac_rerr_posei;
         J_e_pi.block(6, 0, 3, 6) = Jac_verr_posei;

         *H1 = J_e_pi;

         // std::cout << "*H1: \n" << *H1 << std::endl;
      }
      
      if (H2)
      {
         Matrix123 J_e_v;
         J_e_v.setZero();
         Matrix33 Jac_perr_veli  = - Matrix33::Identity() * dt_;
         Matrix33 Jac_verr_v     = - Matrix33::Identity();
         J_e_v.block(0, 0, 3, 3) =   Jac_perr_veli;
         J_e_v.block(6, 0, 3, 3) =   Jac_verr_v;
         
         *H2 = J_e_v;
         // std::cout << "*H2: \n" << *H2 << std::endl;
      }

      if (H3)
      {
         Matrix123 J_e_omage;
         J_e_omage.setZero();
         double a = 1.0f / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
         double b = 1.0f / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
         double c = 1.0f / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);

         Matrix3 d_omega;
         d_omega <<  0,              a * omega_i[2], a * omega_i[1],
                     b * omega_i[2], 0,              b * omega_i[0],
                     c * omega_i[1], c * omega_i[0], 0;

         Matrix33 Jac_r_omega        = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
         Matrix33 Jac_omega_omega    = - I_3x3 + d_omega * dt_;
         J_e_omage.block(3, 0, 3, 3) = Jac_r_omega;
         J_e_omage.block(9, 0, 3, 3) = Jac_omega_omega;

         *H3 = J_e_omage;
         // std::cout << "*H3: \n" << *H3 << std::endl;
      }

      if (H4)
      {
         Matrix124 J_e_input;
         J_e_input.setZero();
         Matrix124 _B;
         _B.setZero();
         
         _B.block(6, 0, 3, 1) = r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0f / dynamics_params_.mass) * dt_;
         _B.block(9, 1, 3, 3) = J_inv * dt_;

         J_e_input = - _B;
         J_e_input.block(0, 0, 3, 1) = - r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0f / dynamics_params_.mass) * dt_ * dt_ * 0.5;
         *H4 = J_e_input;
         // std::cout << "*H4: \n" << *H4 << std::endl;
      }

      if (H5)
      {
         Matrix126 J_e_posej;
         J_e_posej.setZero();
         J_e_posej.block(0, 0, 3, 6) = jac_t_posej;
         J_e_posej.block(3, 0, 3, 6) = J_rerr_rbj * jac_r_posej;

         *H5 = J_e_posej;
         // std::cout << "*H5: \n" << *H5 << std::endl;
      }
      if (H6)
      {
         Matrix123 J_e_vj;
         J_e_vj.setZero();
         J_e_vj.block(6, 0, 3, 3) = Matrix33::Identity();
         *H6 = J_e_vj;
         // std::cout << "*H6: \n" << *H6 << std::endl;
      }

      if (H7)
      {
         Matrix123 J_e_omagej;

         J_e_omagej.setZero();
         J_e_omagej.block(9, 0, 3, 3) = Matrix33::Identity();
         *H7 = J_e_omagej;
         // std::cout << "*H7: \n" << *H7 << std::endl;
      }

      err.head(3) = p_err;
      err.block(3, 0, 3, 1) = rot_err;
      err.block(6, 0, 3, 1) = vel_err;
      err.tail(3) = omega_err;
      // std::cout << "factor error: " << err.transpose() << std::endl;
      return err;
   };

   // Dynamics Factor based on thrust and moments.
   
   DynamicsFactorFullTM::DynamicsFactorFullTM(Key p_i, Key vel_i, Key omega_i, Key tm_ij,
                                  Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, omega_i, tm_ij, p_j, vel_j, omega_j),
         dt_(dt){};

   Vector DynamicsFactorFullTM::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, 
                                        const gtsam::Vector3 &omega_i, const gtsam::Vector6 &thrust_torque,
                                        const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                                        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                        boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                        boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                                        boost::optional<Matrix &> H7) const
   {
      gtsam::Vector12 err;
      Matrix36 jac_t_posei, jac_t_posej;
      Matrix36 jac_r_posei, jac_r_posej;

      const Point3 p_w_i = pos_i.translation(jac_t_posei);
      const Rot3 r_w_bi  = pos_i.rotation(jac_r_posei);
      const Point3 p_w_j = pos_j.translation(jac_t_posej);
      const Rot3 r_w_bj  = pos_j.rotation(jac_r_posej);

      // position rotation velocity error
      gtsam::Vector3 p_err = p_w_j - (p_w_i + vel_i * dt_);
      gtsam::Matrix33 J_rerr_rbj, J_rbi;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
      gtsam::Vector3 vel_err = vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(thrust_torque.head(3) / dynamics_params_.mass)) * dt_);

      // omage error
      gtsam::Matrix3 J, J_inv;
      J << dynamics_params_.Ixx, 0, 0,
           0, dynamics_params_.Iyy, 0,
           0, 0, dynamics_params_.Izz;
      J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
               0, 1.0 / dynamics_params_.Iyy, 0,
               0, 0, 1.0 / dynamics_params_.Izz;
      gtsam::Vector3 omega_err = omega_j - omega_i - J_inv * (thrust_torque.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;

      if (H1)
      {
         Matrix33 Jac_perr_p = -Matrix33::Identity();
         Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
         Matrix33 Jac_verr_r = r_w_bi.matrix() * skewSymmetric(thrust_torque.head(3) / dynamics_params_.mass) * dt_;

         Matrix36 Jac_perr_posei = Jac_perr_p * jac_t_posei;
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
         double a = 1.0 / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
         double b = 1.0 / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
         double c = 1.0 / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);
            
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
         Matrix126 J_e_input;
         J_e_input.setZero();
         Matrix126 _B;
         _B.setZero();
         _B.block(6, 0, 3, 3) = r_w_bi.matrix() * gtsam::Matrix3::Identity() / dynamics_params_.mass * dt_;
         _B.block(9, 3, 3, 3) = J_inv * dt_;

         J_e_input = -_B;
         *H4 = J_e_input;
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

      err.head(3) = p_err;
      err.block(3, 0, 3, 1) = rot_err;
      err.block(6, 0, 3, 1) = vel_err;
      err.tail(3) = omega_err;

      return err;
   };


   // Force and moments between factor.
   BetForceMoments::BetForceMoments(Key input_i, Key input_j, const SharedNoiseModel &model)
       : Base(model, input_i, input_j){};

   Vector BetForceMoments::evaluateError(const gtsam::Vector4 &input_i, const gtsam::Vector4 &input_j,
                        boost::optional<Matrix &> H1, boost::optional<Matrix &> H2
                        ) const
   {
      gtsam::Vector4 err;
      err = input_j - input_i;
      if(H1)
      {
         *H1 = - gtsam::Matrix4::Identity();
      }
      if(H2)
      {
         *H2 = gtsam::Matrix4::Identity();
      }

      return err;
   }

   Vector ControlLimitFactor::evaluateError(const gtsam::Vector4 &input, boost::optional<Matrix &> H1) const
   {
      gtsam::Vector4 error;
      gtsam::Matrix4 jac;
      jac.setZero();

      for(uint i = 0; i < 4; i++)
      {
         if(input[i] >= low_ + thr_ && input[i] <= high_ - thr_)
         {
            error(i) = 0;
            jac(i,i) = 1;
         }
         else if(input[i] < low_ + thr_)
         {
            error(i) = alpha_ * (low_ + thr_ - input[i]);
            jac(i,i) = - alpha_;
         }
         else
         {
            error(i) = alpha_ * (input[i] - high_ + thr_);
            jac(i,i) = alpha_;
         }
      }
      if(H1)
      {
         *H1 = jac;
      }
      return error;
   }

}