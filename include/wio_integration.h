#pragma once

#include <iostream>
#include "gtsam_wrapper.h"

using namespace gtsam;

namespace wio
{
    typedef Eigen::Matrix<double, 12, 3> Matrix12_3;
    typedef Eigen::Matrix<double, 12, 12> Matrix1212;
    typedef Eigen::Matrix<double, 12, 6> Matrix12_6;

    class GTSAM_EXPORT WheelPreintegration : public PreintegrationBase
    {

    protected:
        /**
         * Preintegrated navigation state, as a 9D vector on tangent space at frame i
         * Order is: theta, position, velocity
         */
        Vector12 preintegrated_;
        Matrix12_3 preintegrated_H_biasAcc_;   ///< Jacobian of preintegrated_ w.r.t. acceleration bias
        Matrix12_3 preintegrated_H_biasOmega_; ///< Jacobian of preintegrated_ w.r.t. angular rate bias
        /*
         * jacbian of wheel pim with bRo
         */
        Matrix3 jac_wheel_bRo_;
        /// Default constructor for serialization
        WheelPreintegration()
        {
            resetIntegration();
        }

    public:
        /**
         *  Constructor, initializes the variables in the base class
         *  @param p    Parameters, typically fixed in a single application
         *  @param bias Current estimate of acceleration and rotation rate biases
         */
        WheelPreintegration(const boost::shared_ptr<Params> &p,
                            const imuBias::ConstantBias &biasHat = imuBias::ConstantBias());
        void resetIntegration() override;

        /// Virtual destructor
        virtual ~WheelPreintegration()
        {
        }

        // Update integrated vector on tangent manifold preintegrated with acceleration
        // Static, functional version.
        Vector12 UpdatePreintegrated(const Vector3 &a_body,
                                     const Vector3 &w_body,
                                     const Vector3 &wheel_speed, const double dt,
                                     const Vector12 &preintegrated,
                                     const Rot3 &bRo,
                                     OptionalJacobian<12, 12> A = boost::none,
                                     OptionalJacobian<12, 3> B = boost::none,
                                     OptionalJacobian<12, 3> C = boost::none,
                                     OptionalJacobian<12, 3> D = boost::none);
        /**
         *  Update preintegrated measurements and get derivatives
         * It takes measured quantities in the j frame
         * Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
         */
        void update(const Vector3 &measuredAcc, const Vector3 &measuredOmega,
                    const Vector3 &measuredWheelspeed,
                    const double dt, const Rot3 &bRo, Matrix1212 *A, Matrix12_3 *B, Matrix12_3 *C, Matrix12_3 *D);

        void update(const Vector3 &measuredAcc, const Vector3 &measuredOmega,
                    const double dt, Matrix9 *A, Matrix93 *B, Matrix93 *C) override;

        Vector9 biasCorrectedDelta(const imuBias::ConstantBias &bias_i,
                                   OptionalJacobian<9, 6> H = boost::none) const override;

        Vector3 biasCorrectedWheelDelta(const imuBias::ConstantBias &bias_i,
                                        OptionalJacobian<3, 6> H = boost::none) const;

        Vector3 deltaPij() const override { return preintegrated_.segment<3>(3); }
        Vector3 deltaVij() const override { return preintegrated_.segment<3>(6); }
        Rot3 deltaRij() const override { return Rot3::Expmap(theta()); }
        NavState deltaXij() const override { return NavState().retract(preintegrated_.head<9>()); }

        const Vector12 &preintegrated() const { return preintegrated_; }
        Vector3 theta() const { return preintegrated_.head<3>(); }
        const Matrix12_3 &preintegrated_H_biasAcc() const { return preintegrated_H_biasAcc_; }
        const Matrix12_3 &preintegrated_H_biasOmega() const { return preintegrated_H_biasOmega_; }

        const Vector3 WheelPim() const { return preintegrated_.tail(3); }
        const Matrix3 &Jac_Wheel_bRo() const { return jac_wheel_bRo_; }

    private:
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace wio