#include "quadrotor_simulator/Dynamics_control_factor.h"

namespace UAVFactor
{
   DynamicsFactor::DynamicsFactor(Key p_i, Key vel_i, Key omega_i, Key input_i,
                                  Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j),
         dt_(dt){};

   Vector DynamicsFactor::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
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

   Vector DynamicsFactorTm::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
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

   Vector DynamicsFactorFullTM::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector6 &thrust_torque,
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