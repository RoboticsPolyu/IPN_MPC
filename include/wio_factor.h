#pragma once
#include "wio_integration.h"

namespace wio
{
    class GTSAM_EXPORT PreintegratedImuWheelMeasurements : public WheelPreintegration
    {
        friend class WheelImuFactor;
        friend class WheelImuFactor2;

    protected:
        Matrix1212 preintMeasCov_; ///< COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY]
                                   ///< (first-order propagation from *measurementCovariance*).
        Rot3 bRo_;
        Matrix3 oCov_; //wheel

    public:
        // /// Default constructor for serialization and Cython wrapper
        PreintegratedImuWheelMeasurements()
        {
            preintMeasCov_.setZero();
        }

        /**
         *  Constructor, initializes the class with no measurements
         *  @param bias Current estimate of acceleration and rotation rate biases
         *  @param p    Parameters, typically fixed in a single application
         */
        PreintegratedImuWheelMeasurements(const boost::shared_ptr<PreintegrationParams> &p,
                                          const imuBias::ConstantBias &biasHat = imuBias::ConstantBias(),
                                          const Rot3 &bRo = Rot3(Matrix3::Identity()),
                                          const Matrix3 &oCov = Matrix3::Zero())
            : WheelPreintegration(p, biasHat), bRo_(bRo), oCov_(oCov)
        {
            preintMeasCov_.setZero();
        }

        /**
         *  Construct preintegrated directly from members: base class and preintMeasCov
         *  @param base               WheelPreintegration instance
         *  @param preintMeasCov      Covariance matrix used in noise model.
         */
        PreintegratedImuWheelMeasurements(const WheelPreintegration &base, const Matrix1212 &preintMeasCov)
            : WheelPreintegration(base),
              preintMeasCov_(preintMeasCov)
        {
        }

        /// Virtual destructor
        virtual ~PreintegratedImuWheelMeasurements()
        {
        }

        /// print
        void print(const std::string &s = "Preintegrated Measurements:") const override;

        /// Re-initialize PreintegratedImuWheelMeasurements
        void resetIntegration() override;

        void reset_wheel_extrinsic(const Rot3 &bRo)
        {
            bRo_ = bRo;
        }
        const Rot3 bRo() const { return bRo_; }

        void integrateMeasurement(const Vector3 &measuredAcc,
                                  const Vector3 &measuredOmega,
                                  const Vector3 &measuredWheelspeed,
                                  const double dt);

        /// Return pre-integrated measurement covariance
        Matrix preintMeasCov() const { return preintMeasCov_; }

        /**
         * Add a single IMU measurement to the preintegration.
         * @param measuredAcc Measured acceleration (in body frame, as given by the sensor)
         * @param measuredOmega Measured angular velocity (as given by the sensor)
         * @param dt Time interval between this and the last IMU measurement
         */
        void integrateMeasurement(const Vector3 &measuredAcc,
                                  const Vector3 &measuredOmega, const double dt) override;

        void Predict(const Pose3 &pose_i, const Vector3 &vel_i,
                     Pose3 &pose_j, Vector3 &vel_j, const imuBias::ConstantBias &bias_i);

    private:
    };

    class GTSAM_EXPORT WheelImuFactor : public gtsam_wrapper::NoiseModelFactor7<Pose3, Vector3, Pose3, Vector3,
                                                                                imuBias::ConstantBias, Rot3, Vector3>
    {
    public:
        typedef boost::shared_ptr<ImuFactor> shared_ptr;

        WheelImuFactor() {}

        WheelImuFactor(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias, Key b_r_o, Key b_p_o,
                       const PreintegratedImuWheelMeasurements &pim);

        virtual ~WheelImuFactor()
        {
        }

        const PreintegratedImuWheelMeasurements &preintegratedMeasurements() const
        {
            return _PIM_;
        }

        Vector evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                             const Pose3 &pose_j, const Vector3 &vel_j,
                             const imuBias::ConstantBias &bias_i,
                             const Rot3 &bRo, const Vector3 &bPo,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;

    private:
        typedef WheelImuFactor This;
        typedef gtsam_wrapper::NoiseModelFactor7<Pose3, Vector3, Pose3, Vector3,
                                                 imuBias::ConstantBias, Rot3, Vector3>
            Base;
        PreintegratedImuWheelMeasurements _PIM_;
    };

    class GTSAM_EXPORT WheelImuFactor2 : public NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3,
                                                                  imuBias::ConstantBias, Rot3>
    {
    public:
        typedef boost::shared_ptr<ImuFactor> shared_ptr;

        WheelImuFactor2() {}
        WheelImuFactor2(Key pose_i, Key vel_i, Key pose_j, Key vel_j, Key bias, Key b_r_o,
                        const PreintegratedImuWheelMeasurements &pim, Vector3 &bPo_);

        virtual ~WheelImuFactor2()
        {
        }

        const PreintegratedImuWheelMeasurements &preintegratedMeasurements() const
        {
            return _PIM_;
        }

        Vector evaluateError(const Pose3 &pose_i, const Vector3 &vel_i,
                             const Pose3 &pose_j, const Vector3 &vel_j,
                             const imuBias::ConstantBias &bias_i,
                             const Rot3 &bRo,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none) const;

    private:
        typedef WheelImuFactor This;
        typedef NoiseModelFactor6<Pose3, Vector3, Pose3, Vector3,
                                  imuBias::ConstantBias, Rot3>
            Base;
        PreintegratedImuWheelMeasurements _PIM_;
        Vector3 bPo_;
    };

}; // namespace wio
