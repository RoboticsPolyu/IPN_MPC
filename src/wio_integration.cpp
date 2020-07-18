#include "wio_integration.h"

namespace wio
{
  WheelPreintegration::WheelPreintegration(const boost::shared_ptr<Params> &p,
                                           const Bias &biasHat)
      : PreintegrationBase(p, biasHat)
  {
    resetIntegration();
  }

  void WheelPreintegration::resetIntegration()
  {
    deltaTij_ = 0.0;
    preintegrated_.setZero();
    preintegrated_H_biasAcc_.setZero();
    preintegrated_H_biasOmega_.setZero();
    jac_wheel_bRo_.setZero();
  }

  //------------------------------------------------------------------------------
  // See extensive discussion in ImuFactor.lyx
  Vector12 WheelPreintegration::UpdatePreintegrated(const Vector3 &a_body,
                                                    const Vector3 &w_body,
                                                    const Vector3 &wheel_speed, double dt, const Vector12 &preintegrated,
                                                    const Rot3 &bRo,
                                                    OptionalJacobian<12, 12> A, OptionalJacobian<12, 3> B,
                                                    OptionalJacobian<12, 3> C, OptionalJacobian<12, 3> D)
  {
    const auto theta = preintegrated.segment<3>(0);
    const auto position = preintegrated.segment<3>(3);
    const auto velocity = preintegrated.segment<3>(6);
    const auto wheel_odo = preintegrated.segment<3>(9);

    // This functor allows for saving computation when exponential map and its
    // derivatives are needed at the same location in so<3>
    so3::DexpFunctor local(theta);

    // Calculate exact mean propagation
    Matrix3 w_tangent_H_theta, invH;
    const Vector3 w_tangent = // angular velocity mapped back to tangent space
        local.applyInvDexp(w_body, A ? &w_tangent_H_theta : 0, C ? &invH : 0);
    const Rot3 R(local.expmap());
    const Vector3 a_nav = R * a_body;
    const double dt22 = 0.5 * dt * dt;

    Vector12 preintegratedPlus;
    preintegratedPlus <<                         // new preintegrated vector:
        theta + w_tangent * dt,                  // theta
        position + velocity * dt + a_nav * dt22, // position
        velocity + a_nav * dt,                   // velocity
        wheel_odo + R * bRo * wheel_speed * dt;  // wheel odometry

    if (A)
    {
      // Exact derivative of R*a with respect to theta:
      const Matrix3 a_nav_H_theta = R.matrix() * skewSymmetric(-a_body) * local.dexp();

      A->setIdentity();
      A->block<3, 3>(0, 0).noalias() += w_tangent_H_theta * dt; // theta
      A->block<3, 3>(3, 0) = a_nav_H_theta * dt22;              // position wrpt theta...
      A->block<3, 3>(3, 6) = I_3x3 * dt;                        // .. and velocity
      A->block<3, 3>(6, 0) = a_nav_H_theta * dt;                // velocity wrpt theta

      const Vector3 wheelspeed_body = bRo.matrix() * wheel_speed;
      A->block<3, 3>(9, 0) = R.matrix() * skewSymmetric(-wheelspeed_body) * local.dexp() * dt; // wheel odo wrpt theta
    }
    if (B)
    {
      B->block<3, 3>(0, 0) = Z_3x3;
      B->block<3, 3>(3, 0) = R.matrix() * dt22;
      B->block<3, 3>(6, 0) = R.matrix() * dt;
      B->block<3, 3>(9, 0) = Z_3x3;
    }
    if (C)
    {
      C->block<3, 3>(0, 0) = invH * dt;
      C->block<3, 3>(3, 0) = Z_3x3;
      C->block<3, 3>(6, 0) = Z_3x3;
      C->block<3, 3>(9, 0) = Z_3x3;
    }
    if (D)
    {
      D->block<3, 3>(0, 0) = Z_3x3;
      D->block<3, 3>(3, 0) = Z_3x3;
      D->block<3, 3>(6, 0) = Z_3x3;
      D->block<3, 3>(9, 0) = R.matrix() * bRo.matrix() * dt;
    }

    jac_wheel_bRo_ += -R.matrix() * bRo.matrix() * skewSymmetric(wheel_speed) * dt;
    // std::cout << "**********************************************************" << std::endl;
    // cout << "      jac_wheel_bRo \n"
    //      << jac_wheel_bRo_ << "\n";
    // cout << "      preintegratedPlus \n"
    //      << preintegratedPlus << "\n";
    // cout << " bRo:\n"
    //      << bRo << " ,wheel velocity:\n"
    //      << wheel_speed << "\n";
    // std::cout << "**********************************************************" << std::endl;

    return preintegratedPlus;
  }

  void WheelPreintegration::update(const Vector3 &measuredAcc,
                                   const Vector3 &measuredOmega, const double dt, Matrix9 *A, Matrix93 *B,
                                   Matrix93 *C)
  {
  }

  void WheelPreintegration::update(const Vector3 &measuredAcc,
                                   const Vector3 &measuredOmega,
                                   const Vector3 &measuredWheelspeed, const double dt, const Rot3 &bRo,
                                   Matrix1212 *A, Matrix12_3 *B,
                                   Matrix12_3 *C, Matrix12_3 *D)
  {
    // Correct for bias in the sensor frame
    Vector3 acc = biasHat_.correctAccelerometer(measuredAcc);
    Vector3 omega = biasHat_.correctGyroscope(measuredOmega);

    // Possibly correct for sensor pose
    Matrix3 D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega;
    if (p().body_P_sensor)
      boost::tie(acc, omega) = correctMeasurementsBySensorPose(acc, omega,
                                                               D_correctedAcc_acc, D_correctedAcc_omega, D_correctedOmega_omega);

    // Do update
    deltaTij_ += dt;
    preintegrated_ = UpdatePreintegrated(acc, omega, measuredWheelspeed, dt, preintegrated_, bRo, A, B, C, D);

    if (p().body_P_sensor)
    {
      // More complicated derivatives in case of non-trivial sensor pose
      *C *= D_correctedOmega_omega;
      if (!p().body_P_sensor->translation().isZero())
        *C += *B * D_correctedAcc_omega;
      *B *= D_correctedAcc_acc; // NOTE(frank): needs to be last
    }

    // new_H_biasAcc = new_H_old * old_H_biasAcc + new_H_acc * acc_H_biasAcc
    // where acc_H_biasAcc = -I_3x3, hence
    // new_H_biasAcc = new_H_old * old_H_biasAcc - new_H_acc
    preintegrated_H_biasAcc_ = (*A) * preintegrated_H_biasAcc_ - (*B);

    // new_H_biasOmega = new_H_old * old_H_biasOmega + new_H_omega * omega_H_biasOmega
    // where omega_H_biasOmega = -I_3x3, hence
    // new_H_biasOmega = new_H_old * old_H_biasOmega - new_H_omega
    preintegrated_H_biasOmega_ = (*A) * preintegrated_H_biasOmega_ - (*C);
  }

  Vector3 WheelPreintegration::biasCorrectedWheelDelta(const imuBias::ConstantBias &bias_i,
                                                       OptionalJacobian<3, 6> H) const
  {
    const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
    const Vector12 biasCorrected = preintegrated() + preintegrated_H_biasAcc_ * biasIncr.accelerometer() +
                                   preintegrated_H_biasOmega_ * biasIncr.gyroscope();
    Matrix12_6 H_error_bias;
    H_error_bias.block<12, 3>(0, 0) = preintegrated_H_biasAcc_;
    H_error_bias.block<12, 3>(0, 3) = preintegrated_H_biasOmega_;

    if (H)
    {
      *H = H_error_bias.block<3, 6>(9, 0);
    }
    // std::cout << "bias corrected wheel odo: " << biasCorrected.tail(3) - preintegrated().tail(3) << std::endl;
    return biasCorrected.tail(3);
  }

  Vector9 WheelPreintegration::biasCorrectedDelta(const imuBias::ConstantBias &bias_i,
                                                  OptionalJacobian<9, 6> H) const
  {
    // We correct for a change between bias_i and the biasHat_ used to integrate
    // This is a simple linear correction with obvious derivatives
    const imuBias::ConstantBias biasIncr = bias_i - biasHat_;
    const Vector12 biasCorrected = preintegrated() + preintegrated_H_biasAcc_ * biasIncr.accelerometer() +
                                   preintegrated_H_biasOmega_ * biasIncr.gyroscope();
    Matrix12_6 H_error_bias;
    H_error_bias.block<12, 3>(0, 0) = preintegrated_H_biasAcc_;
    H_error_bias.block<12, 3>(0, 3) = preintegrated_H_biasOmega_;

    if (H)
    {
      *H = H_error_bias.block<9, 6>(0, 0);
    }

    return biasCorrected.head(9);
  }
} // namespace wio
