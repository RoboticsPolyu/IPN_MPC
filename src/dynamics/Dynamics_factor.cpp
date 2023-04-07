#include "quadrotor_simulator/Dynamics_factor.h"

namespace UAV_Factor
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
      const Rot3 r_w_bi = pos_i.rotation(jac_r_posei);
      const Point3 p_w_j = pos_j.translation(jac_t_posej);
      const Rot3 r_w_bj = pos_j.rotation(jac_r_posej);

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
      gtsam::Matrix33 J_rerr_rbj, J_rbi;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
      gtsam::Vector3 vel_err = vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(gtsam::Vector3(0, 0, T_mb[0] / dynamics_params_.mass))) * dt_);

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
         double a = -1.0 / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
         double b = -1.0 / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
         double c = -1.0 / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);
         Matrix3 d_omega;
         d_omega << 0, a * omega_i[2], a * omega_i[1],
             b * omega_i[2], 0, b * omega_i[0],
             c * omega_i[1], c * omega_i[0], 0;

         Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
         Matrix33 Jac_omega_omega = -d_omega * dt_;
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
         _B.block(6, 0, 3, 1) = r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0 / dynamics_params_.mass) * dt_;
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

   DynamicsFactorfm::DynamicsFactorfm(Key p_i, Key vel_i, Key omega_i, Key input_i,
                                  Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j),
         dt_(dt){};

   Vector DynamicsFactorfm::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
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
      const Rot3 r_w_bi = pos_i.rotation(jac_r_posei);
      const Point3 p_w_j = pos_j.translation(jac_t_posej);
      const Rot3 r_w_bj = pos_j.rotation(jac_r_posej);

      // force and moment

      // position rotation velocity error
      gtsam::Vector3 p_err = p_w_j - (p_w_i + vel_i * dt_);
      gtsam::Matrix33 J_rerr_rbj, J_rbi;
      gtsam::Vector3 rot_err = Rot3::Logmap(r_w_bj.between(r_w_bi.compose(Rot3::Expmap(omega_i * dt_), J_rbi), J_rerr_rbj));
      gtsam::Vector3 vel_err = vel_j - (vel_i + (-gtsam::Vector3(0, 0, dynamics_params_.g) + r_w_bi.rotate(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass))) * dt_);

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
      gtsam::Vector3 omega_err = omega_j - omega_i - J_inv * (input_i.tail(3) - skewSymmetric(omega_i) * J * omega_i) * dt_;
      // std::cout << "Dynmaics Factor omage_err: \n" << omega_err << std::endl;

      if (H1)
      {
         Matrix33 Jac_perr_p = -Matrix33::Identity();
         Matrix33 Jac_rerr_r = J_rbi; // Matrix33::Identity() - skewSymmetric(omega_i) * dt_;
         Matrix33 Jac_verr_r = r_w_bi.matrix() * skewSymmetric(gtsam::Vector3(0, 0, input_i[0] / dynamics_params_.mass)) * dt_;

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
         double a = -1.0 / dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
         double b = -1.0 / dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
         double c = -1.0 / dynamics_params_.Izz * (dynamics_params_.Iyy - dynamics_params_.Ixx);
         Matrix3 d_omega;
         d_omega << 0, a * omega_i[2], a * omega_i[1],
             b * omega_i[2], 0, b * omega_i[0],
             c * omega_i[1], c * omega_i[0], 0;

         Matrix33 Jac_r_omega = SO3::ExpmapDerivative(omega_i * dt_) * dt_;
         Matrix33 Jac_omega_omega = -d_omega * dt_;
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
         _B.block(6, 0, 3, 1) = r_w_bi.matrix() * gtsam::Vector3(0, 0, 1.0 / dynamics_params_.mass) * dt_;
         _B.block(9, 1, 3, 3) = J_inv * dt_;

         J_e_input = -_B;
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

   /*************Dynmaics Factor2 *************/
   DynamicsFactor2::DynamicsFactor2(Key x_i, Key input_i, Key x_j, float dt, const SharedNoiseModel &model)
       : Base(model, x_i, input_i, x_j),
         dt_(dt){

         };

   Vector DynamicsFactor2::evaluateError(const gtsam::Vector12 &x_i, const gtsam::Vector4 &input_i, const gtsam::Vector12 &x_j,
                                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                         boost::optional<Matrix &> H3) const
   {
      gtsam::Vector12 error;
      gtsam::Vector4 input = input_i;
      Matrix4 K_ = _K(input);
      gtsam::Vector Fmb = K_ * input_i;
      gtsam::Vector3 theta = x_i.block(6, 0, 3, 0);
      gtsam::Vector3 omega = x_i.block(9, 0, 3, 0);
      Mat12 A_ = _A(omega, theta, Fmb[0]);
      Matrix124 B_ = _B(theta);

      error = x_j - (A_ * dt_ + Mat12::Identity()) * x_i - B_ * K_ * dt_ * input_i;

      if (H1)
      {
         *H1 = A_;
      }
      if (H2)
      {
         *H2 = B_ * K_;
      }
      if (H3)
      {
         *H3 = Mat12::Identity();
      }

      return error;
   };

   Mat12 DynamicsFactor2::_A(gtsam::Vector3 &omega, gtsam::Vector3 &theta, float trust) const
   {
      Mat12 _A;
      gtsam::Rot3 rot = gtsam::Rot3::Expmap(theta);
      _A.setZero();
      _A.block(0, 3, 3, 3) = Matrix3::Identity();
      _A.block(3, 6, 3, 3) = rot.matrix() * skewSymmetric(gtsam::Vector3(0, 0, trust)) / dynamics_params_.mass;
      _A.block(6, 6, 3, 3) = -gtsam::skewSymmetric(omega);
      _A.block(6, 9, 3, 3) = gtsam::skewSymmetric(theta);

      double a = dynamics_params_.Ixx * (dynamics_params_.Izz - dynamics_params_.Iyy);
      double b = dynamics_params_.Iyy * (dynamics_params_.Ixx - dynamics_params_.Izz);
      double c = dynamics_params_.Izz * (dynamics_params_.Izz - dynamics_params_.Ixx);
      Matrix3 d_omega;
      d_omega << 0, a * omega[2], a * omega[1],
          b * omega[2], 0, b * omega[1],
          c * omega[1], c * omega[0], 0;

      _A.block(9, 9, 3, 3) = d_omega;

      return _A;
   }

   Matrix124 DynamicsFactor2::_B(gtsam::Vector3 &theta) const
   {
      Matrix124 _B;
      _B.setZero();

      gtsam::Rot3 rot = gtsam::Rot3::Expmap(theta);
      _B.block(3, 0, 3, 1) = -rot.matrix() * gtsam::Vector3(0, 0, 1.0 / dynamics_params_.mass);
      gtsam::Matrix3 J_inv;
      J_inv << 1.0 / dynamics_params_.Ixx, 0, 0,
          0, 1.0 / dynamics_params_.Iyy, 0,
          0, 0, 1.0 / dynamics_params_.Izz;

      _B.block(9, 1, 3, 3) = J_inv;
      return _B;
   }

   Matrix4 DynamicsFactor2::_K(gtsam::Vector4 &input) const
   {
      Matrix4 K1, K2;
      K1 << dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f, dynamics_params_.k_f,
          0, 0, dynamics_params_.arm_length * dynamics_params_.k_f, -dynamics_params_.arm_length * dynamics_params_.k_f,
          -dynamics_params_.arm_length * dynamics_params_.k_f, dynamics_params_.arm_length * dynamics_params_.k_f, 0, 0,
          dynamics_params_.k_m, dynamics_params_.k_m, -dynamics_params_.k_m, -dynamics_params_.k_m;
      K2 << 2 * input[0], 0, 0, 0,
          0, 2 * input[1], 0, 0,
          0, 0, 2 * input[2], 0,
          0, 0, 0, 2 * input[3];
      return K1 * K2;
   }

}