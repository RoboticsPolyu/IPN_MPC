#ifndef __DYNAMICS_FACTOR_H__
#define __DYNAMICS_FACTOR_H__

#include "Quadrotor_SO3.h"
#include "gtsam_wrapper.h"
#include <vector>

using namespace gtsam_wrapper;

namespace uav_factor
{
    typedef Eigen::Matrix<double, 12, 12> Mat12;
    typedef Eigen::Matrix<double, 12, 4> Matrix124;
    typedef QuadrotorSimulator_SO3::Quadrotor::State UAV_State;
    struct DynamicsParams
    {
        float g = 9.81;
        float mass = 0.98;
        double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
        double k_f = 8.98132e-9;
        double prop_radius_ = 0.062;
        double k_m = 0.07 * (3 * prop_radius_) * k_f;
        float min_rpm = 1200;
        float max_rpm = 35000;
        float arm_length = 0.26;
    };

    class Dynamics
    {

    public:
        // // km_ = 2.5e-9; // from Nate
        // // km = (Cq/Ct)*Dia*kf
        // // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
        // km_ = 0.07 * (3 * prop_radius_) * kf_; // z-moment k-gain

        Dynamics()
        {
            MeasCov_.setZero();
        }

        Dynamics(std::vector<UAV_State> &state_refs, std::vector<gtsam::Vector4> &input_refs)
            : state_refs_(state_refs), input_refs_(input_refs)
        {
        }

        Mat12 MeasCov_; // dx+1's probability
        Mat12 Jac_dx_;
        Matrix124 Jac_du_;

    private:
        gtsam::Matrix4 K_;
        std::vector<UAV_State> state_refs_;
        std::vector<gtsam::Vector4> input_refs_;
        DynamicsParams dynmaics_params_;
    };

    /* position velocity rotation angular_velocity control_input*/

    class GTSAM_EXPORT DynamicsFactor : public NoiseModelFactor9<gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3, gtsam::Vector4,
                                                                 gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactor> shared_ptr;

        DynamicsFactor() {}
        DynamicsFactor(Key p_i, Key vel_i, Key rot_i, Key omega_i, Key input_i, Key p_j, Key vel_j, Key rot_j, Key omega_j,
                       const Dynamics &dys);

        virtual ~DynamicsFactor()
        {
        }

        const Dynamics &dynamics() const
        {
            return _DYS_;
        }

        Vector evaluateError(const gtsam::Vector3 &pos_i, const gtsam::Vector3 &vel_i, gtsam::Rot3 &rot_i, gtsam::Vector3 &omega_i, gtsam::Vector4 &input_i,
                             const gtsam::Vector3 &pos_j, const gtsam::Vector3 &vel_j, gtsam::Rot3 &rot_j, gtsam::Vector3 &omega_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none, boost::optional<Matrix &> H8 = boost::none,
                             boost::optional<Matrix &> H9 = boost::none) const;

    private:
        typedef DynamicsFactor This;
        typedef NoiseModelFactor9<gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3, gtsam::Vector4,
                                  gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3>
            Base;
        Dynamics _DYS_;
    };

    /* dynamics factor basic version*/

    class GTSAM_EXPORT DynamicsFactor2 : public NoiseModelFactor3<gtsam::Vector12, gtsam::Vector4, gtsam::Vector12>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactor> shared_ptr;

        DynamicsFactor2() {}
        DynamicsFactor2(Key x_i, Key input_i, Key x_j, float dt, const SharedNoiseModel &model);

        virtual ~DynamicsFactor2()
        {
        }

        Vector evaluateError(const gtsam::Vector12 &x_i, gtsam::Vector4 &input_i, gtsam::Vector12 &x_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none) const
        {
            gtsam::Vector12 error;
            Matrix4 K_ = _K(input_i);
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

    private:
        typedef DynamicsFactor2 This;
        typedef NoiseModelFactor3<gtsam::Vector12, gtsam::Vector4, gtsam::Vector12>
            Base;

        DynamicsParams dynmaics_params_;

        float dt_;

        Mat12 _A(gtsam::Vector3 &omega, gtsam::Vector3 &theta, float trust) const
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

        Matrix124 _B(gtsam::Vector3 theta) const
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

        Matrix4 _K(gtsam::Vector4 &input) const
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
    };
}

#endif // __DYNAMICS_FACTOR_H__