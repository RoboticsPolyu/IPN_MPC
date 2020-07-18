#include "wio_factor.h"

namespace wio
{
    using namespace std;

    //------------------------------------------------------------------------------
    // Inner class PreintegratedImuWheelMeasurements
    //------------------------------------------------------------------------------
    void PreintegratedImuWheelMeasurements::print(const string &s) const
    {
        WheelPreintegration::print(s);
        cout << "    preintMeasCov \n"
             << preintMeasCov_ << endl;
        cout << "    jac wheel bRo \n"
             << jac_wheel_bRo_ << endl;
        cout << "    preintegration \n"
             << preintegrated() << endl;
        cout << "    rRo \n"
             << bRo() << "\n";
    }

    //------------------------------------------------------------------------------
    void PreintegratedImuWheelMeasurements::resetIntegration()
    {
        WheelPreintegration::resetIntegration();
        preintMeasCov_.setZero();
    }

    //------------------------------------------------------------------------------
    void PreintegratedImuWheelMeasurements::integrateMeasurement(
        const Vector3 &measuredAcc, const Vector3 &measuredOmega, const Vector3 &measuredWheelspeed,
        double dt)
    {
        if (dt <= 0)
        {
            throw std::runtime_error(
                "PreintegratedImuWheelMeasurements::integrateMeasurement: dt <=0");
        }

        // Update preintegrated measurements (also get Jacobian)
        Matrix1212 A; // overall Jacobian wrt preintegrated measurements (df/dx)
        Matrix12_3 B, C, D;
        WheelPreintegration::update(measuredAcc, measuredOmega, measuredWheelspeed, dt, bRo_, &A, &B, &C, &D);

        // first order covariance propagation:
        // as in [2] we consider a first order propagation that can be seen as a
        // prediction phase in EKF

        // propagate uncertainty
        // TODO(frank): use noiseModel routine so we can have arbitrary noise models.
        const Matrix3 &aCov = p().accelerometerCovariance;
        const Matrix3 &wCov = p().gyroscopeCovariance;
        const Matrix3 &iCov = p().integrationCovariance;
        const Matrix3 &oCov = oCov_;

        // (1/dt) allows to pass from continuous time noise to discrete time noise
        preintMeasCov_ = A * preintMeasCov_ * A.transpose();
        preintMeasCov_.noalias() += B * (aCov / dt) * B.transpose();
        preintMeasCov_.noalias() += C * (wCov / dt) * C.transpose();
        preintMeasCov_.noalias() += D * (oCov / dt) * D.transpose();

        // NOTE(frank): (Gi*dt)*(C/dt)*(Gi'*dt), with Gi << Z_3x3, I_3x3, Z_3x3
        preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
    }

    //------------------------------------------------------------------------------
    void PreintegratedImuWheelMeasurements::integrateMeasurement(
        const Vector3 &measuredAcc, const Vector3 &measuredOmega, double dt)
    {
        std::cout << "unused func: integrate measurement" << std::endl;
    }

    WheelImuFactor::WheelImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias, Key b_r_o, Key b_p_o,
                                   const PreintegratedImuWheelMeasurements &pim)
        : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i, pose_j,
               vel_j, bias, b_r_o, b_p_o),
          _PIM_(pim)
    {
    }

    void PreintegratedImuWheelMeasurements::Predict(const Pose3 &pose_i, const Vector3 &vel_i,
                                                    Pose3 &pose_j, Vector3 &vel_j, const imuBias::ConstantBias &bias_i)
    {
        NavState state_bak(pose_i, vel_i);
        NavState pvb = predict(state_bak, bias_i);
        pose_j = pvb.pose();
        vel_j = pvb.v();
    }

    Vector WheelImuFactor::evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                                         const Pose3 &pose_j, const Vector3 &vel_j,
                                         const imuBias::ConstantBias &bias_i,
                                         const Rot3 &bRo, const Vector3 &bPo,
                                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                         boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                         boost::optional<Matrix &> H5, boost::optional<Matrix &> H6,
                                         boost::optional<Matrix &> H7) const
    {
        // std::cout << "----------------------- imu wheel evalutor ---------------------" << std::endl;
        Matrix96 _H1, _H3, _H5;
        Matrix93 _H2, _H4;
        Vector9 imu_pim_error = _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i, _H1, _H2, _H3, _H4, _H5);

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;

        const Point3 p_w_i = pose_i.translation(jac_t_posei);
        const Rot3 r_w_bi = pose_i.rotation(jac_r_posei);
        const Point3 p_w_j = pose_j.translation(jac_t_posej);
        const Rot3 r_w_bj = pose_j.rotation(jac_r_posej);

        const Rot3 r_bi_w = r_w_bi.inverse();

        Matrix3 H_rjp_rj, H_wpe_rbi_1, H_wpe_rbi_2;
        Matrix3 jac_wheel_bRo = -_PIM_.Jac_Wheel_bRo();

        Vector3 delta_bRo = Rot3::LocalCoordinates(bRo.between(_PIM_.bRo()));

        // std::cout << "linear bRo: "
        //           << Rot3::LocalCoordinates(_PIM_.bRo()) << std::endl;
        // std::cout << "new bRo: \n"
        //           << Rot3::LocalCoordinates(bRo) << std::endl;
        // std::cout << "delat bRo: \n"
        //           << delta_bRo << std::endl;
        // std::cout << "bPo: \n"
        //           << bPo << std::endl;
        // std::cout << "wheel pim: \n"
        //           << _PIM_.WheelPim()
        //           << std::endl;

        Vector3 pose_wheel = r_w_bi.unrotate((p_w_j - p_w_i), H_wpe_rbi_1) - bPo +
                             r_w_bi.unrotate(r_w_bj.rotate(bPo, H_rjp_rj), H_wpe_rbi_2);
        // std::cout << "pose wheel: \n"
        //           << pose_wheel << std::endl;
        Vector3 dog_wheel = jac_wheel_bRo * delta_bRo;
        // std::cout << "dog wheel: \n"
        //           << dog_wheel << std::endl;
        Vector3 wheel_odo = _PIM_.biasCorrectedWheelDelta(bias_i);
        // std::cout << "wheel odo:\n " << wheel_odo << std::endl;
        Vector3 wheel_pim_error = pose_wheel - wheel_odo - dog_wheel; // same as （_PIM_.WheelPim() - jac_wheel_bRo * delta_bRo)

        Matrix33 jac_wheel_bPo = r_bi_w.matrix() * r_w_bj.matrix() - Matrix3::Identity();

        Matrix36 jac_wheel_posei = -r_bi_w.matrix() * jac_t_posei + (H_wpe_rbi_1 + H_wpe_rbi_2) * jac_r_posei;
        Matrix36 jac_wheel_posej = r_bi_w.matrix() * jac_t_posej + r_bi_w.matrix() * H_rjp_rj * jac_r_posej;

        Matrix12_6 H_error_pi, H_error_pj, H_error_bias;
        Matrix12_3 H_error_vi, H_error_vj, H_error_bRo, H_error_bPo;

        H_error_pi.setZero();
        H_error_pj.setZero();
        H_error_vi.setZero();
        H_error_vj.setZero();
        H_error_bias.setZero();
        H_error_bRo.setZero();
        H_error_bPo.setZero();
        H_error_pi.block<9, 6>(0, 0) = _H1;
        H_error_pi.block<3, 6>(9, 0) = jac_wheel_posei;
        H_error_vi.block<9, 3>(0, 0) = _H2;
        H_error_pj.block<9, 6>(0, 0) = _H3;
        H_error_pj.block<3, 6>(9, 0) = jac_wheel_posej;
        H_error_vj.block<9, 3>(0, 0) = _H4;
        H_error_bias.block<9, 6>(0, 0) = _H5;
        H_error_bRo.block<3, 3>(9, 0) = jac_wheel_bRo;
        H_error_bPo.block<3, 3>(9, 0) = jac_wheel_bPo;

        if (H1)
        {
            *H1 = H_error_pi;
        }
        if (H2)
        {
            *H2 = H_error_vi;
        }
        if (H3)
        {
            *H3 = H_error_pj;
        }
        if (H4)
        {
            *H4 = H_error_vj;
        }
        if (H5)
        {
            *H5 = H_error_bias;
        }
        if (H6)
        {
            *H6 = H_error_bRo;
        }
        if (H7)
        {
            *H7 = H_error_bPo;
        }

        Vector12 error;
        error.head(9) = imu_pim_error;
        error.tail(3) = wheel_pim_error;

        // std::cout << "imu wheel error:\n"
        //       << error << std::endl;
        //std::cout << "-------------------------------------------------------------" << std::endl;
        return error;
    }

    WheelImuFactor2::WheelImuFactor2(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias, Key b_r_o,
                                     const PreintegratedImuWheelMeasurements &pim, Vector3 &bPo)
        : Base(noiseModel::Gaussian::Covariance(pim.preintMeasCov_), pose_i, vel_i, pose_j,
               vel_j, bias, b_r_o),
          _PIM_(pim), bPo_(bPo)
    {
    }

    Vector WheelImuFactor2::evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                                          const Pose3 &pose_j, const Vector3 &vel_j,
                                          const imuBias::ConstantBias &bias_i,
                                          const Rot3 &bRo,
                                          boost::optional<Matrix &> H1, boost::optional<Matrix &> H2,
                                          boost::optional<Matrix &> H3, boost::optional<Matrix &> H4,
                                          boost::optional<Matrix &> H5, boost::optional<Matrix &> H6) const
    {
        Matrix96 _H1, _H3, _H5;
        Matrix93 _H2, _H4;
        Vector9 imu_pim_error = _PIM_.computeErrorAndJacobians(pose_i, vel_i, pose_j, vel_j, bias_i, _H1, _H2, _H3, _H4, _H5);

        Matrix36 jac_t_posei, jac_t_posej;
        Matrix36 jac_r_posei, jac_r_posej;

        const Point3 p_w_i = pose_i.translation(jac_t_posei);
        const Rot3 r_w_bi = pose_i.rotation(jac_r_posei);
        const Point3 p_w_j = pose_j.translation(jac_t_posej);
        const Rot3 r_w_bj = pose_j.rotation(jac_r_posej);

        const Rot3 r_bi_w = r_w_bi.inverse();

        Matrix3 H_rjp_rj, H_wpe_rbi_1, H_wpe_rbi_2;
        Matrix3 jac_wheel_bRo = -_PIM_.Jac_Wheel_bRo();

        Vector3 delta_bRo = Rot3::LocalCoordinates(bRo.between(_PIM_.bRo())); // check ?
        Vector3 wheel_pim_error = r_w_bi.unrotate((p_w_j - p_w_i), H_wpe_rbi_1) - bPo_ +
                                  r_w_bi.unrotate(r_w_bj.rotate(bPo_, H_rjp_rj), H_wpe_rbi_2) -
                                  _PIM_.WheelPim() - jac_wheel_bRo * delta_bRo; // same as （_PIM_.WheelPim() - jac_wheel_bRo * delta_bRo)

        Matrix36 jac_wheel_posei = -r_bi_w.matrix() * jac_t_posei + (H_wpe_rbi_1 + H_wpe_rbi_2) * jac_r_posei;
        Matrix36 jac_wheel_posej = r_bi_w.matrix() * jac_t_posej + r_bi_w.matrix() * H_rjp_rj * jac_r_posej;

        Matrix12_6 H_error_pi, H_error_pj, H_error_bias;
        Matrix12_3 H_error_vi, H_error_vj, H_error_bRo, H_error_bPo;

        H_error_pi.setZero();
        H_error_pj.setZero();
        H_error_vi.setZero();
        H_error_vj.setZero();
        H_error_bias.setZero();
        H_error_bRo.setZero();
        H_error_bPo.setZero();
        H_error_pi.block<9, 6>(0, 0) = _H1;
        H_error_pi.block<3, 6>(9, 0) = jac_wheel_posei;
        H_error_vi.block<9, 3>(0, 0) = _H2;
        H_error_pj.block<9, 6>(0, 0) = _H3;
        H_error_pj.block<3, 6>(9, 0) = jac_wheel_posej;
        H_error_vj.block<9, 3>(0, 0) = _H4;
        H_error_bias.block<9, 6>(0, 0) = _H5;
        H_error_bRo.block<3, 3>(9, 0) = jac_wheel_bRo;

        if (H1)
        {
            *H1 = H_error_pi;
        }

        if (H2)
        {
            *H2 = H_error_vi;
        }
        if (H3)
        {
            *H3 = H_error_pj;
        }
        if (H4)
        {
            *H4 = H_error_vj;
        }
        if (H5)
        {
            *H5 = H_error_bias;
        }
        if (H6)
        {
            *H6 = H_error_bRo;
        }

        Vector12 error;
        error.head(9) = imu_pim_error;
        error.tail(3) = wheel_pim_error;

        return error;
    }
} // namespace wio