#include "quadrotor_simulator/Dynamics_factor.h"

namespace uav_factor
{
   DynamicsFactor::DynamicsFactor(Key p_i, Key vel_i, Key omega_i, Key input_i,
                                  Key p_j, Key vel_j, Key omega_j, const SharedNoiseModel &model)
       : Base(model, p_i, vel_i, omega_i, input_i, p_j, vel_j, omega_j){};

   Vector DynamicsFactor::evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                             boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                             boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                             boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                             boost::optional<Matrix &> H7) const
   {
      gtsam::Vector12 err;

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
      _A.block(3, 6, 3, 3) = rot.matrix() * gtsam::Vector3(0, 0, trust);
      _A.block(6, 6, 3, 3) = -gtsam::skewSymmetric(omega);
      _A.block(6, 9, 3, 3) = gtsam::skewSymmetric(theta);

      double a = dynmaics_params_.Ixx * (dynmaics_params_.Izz - dynmaics_params_.Iyy);
      double b = dynmaics_params_.Iyy * (dynmaics_params_.Ixx - dynmaics_params_.Izz);
      double c = dynmaics_params_.Izz * (dynmaics_params_.Izz - dynmaics_params_.Ixx);
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
      _B.block(3, 0, 3, 1) = -rot.matrix() * gtsam::Vector3(0, 0, 1.0 / dynmaics_params_.mass);
      gtsam::Matrix3 J_inv;
      J_inv << 1.0 / dynmaics_params_.Ixx, 0, 0,
          0, 1.0 / dynmaics_params_.Iyy, 0,
          0, 0, 1.0 / dynmaics_params_.Izz;

      _B.block(9, 1, 3, 3) = J_inv;
      return _B;
   }

   Matrix4 DynamicsFactor2::_K(gtsam::Vector4 &input) const
   {
      Matrix4 K1, K2;
      K1 << dynmaics_params_.k_f, dynmaics_params_.k_f, dynmaics_params_.k_f, dynmaics_params_.k_f,
          0, 0, dynmaics_params_.arm_length * dynmaics_params_.k_f, -dynmaics_params_.arm_length * dynmaics_params_.k_f,
          -dynmaics_params_.arm_length * dynmaics_params_.k_f, dynmaics_params_.arm_length * dynmaics_params_.k_f, 0, 0,
          dynmaics_params_.k_m, dynmaics_params_.k_m, -dynmaics_params_.k_m, -dynmaics_params_.k_m;
      K2 << 2 * input[0], 0, 0, 0,
          0, 2 * input[1], 0, 0,
          0, 0, 2 * input[2], 0,
          0, 0, 0, 2 * input[3];
      return K1 * K2;
   }

}