#ifndef __DYNAMICS_PLANNING_FACTOR_H__
#define __DYNAMICS_PLANNING_FACTOR_H__

#include "Dynamics_params.h"
#include "gtsam_wrapper.h"
#include "Quadrotor_SO3.h"

#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor
{    
    typedef QuadrotorSim_SO3::Quadrotor::State UAV_State;


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
        DynamicsParams dynamics_params_;
    };

    /* position velocity rotation angular_velocity control_input*/
    class GTSAM_EXPORT DynamicsFactor : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                                                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactor> shared_ptr;

        DynamicsFactor() {}
        DynamicsFactor(Key p_i, Key vel_i, Key omega_i, Key input_i, Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model);

        virtual ~DynamicsFactor()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;

    private:
        typedef DynamicsFactor This;
        typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                                gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
            Base;

        DynamicsParams dynamics_params_;
        
        float dt_;
    };

    /* dynamics factor basic version*/

    class GTSAM_EXPORT DynamicsFactor2 : public NoiseModelFactor3<gtsam::Vector12, gtsam::Vector4, gtsam::Vector12>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactor2> shared_ptr;

        DynamicsFactor2() {}
        DynamicsFactor2(Key x_i, Key input_i, Key x_j, float dt, const SharedNoiseModel &model);

        virtual ~DynamicsFactor2()
        {
        }

        Vector evaluateError(const gtsam::Vector12 &x_i, const gtsam::Vector4 &input_i, const gtsam::Vector12 &x_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none) const;

    private:
        typedef DynamicsFactor2 This;
        typedef NoiseModelFactor3<gtsam::Vector12, gtsam::Vector4, gtsam::Vector12>
            Base;

        DynamicsParams dynamics_params_;

        float dt_;

        Mat12 _A(gtsam::Vector3 &omega, gtsam::Vector3 &theta, float trust) const;
        Matrix124 _B(gtsam::Vector3 &theta) const;
        Matrix4 _K(gtsam::Vector4 &input) const;
    };


    /* position velocity rotation angular_velocity control_input:Force and Moment*/
    
    class GTSAM_EXPORT DynamicsFactorTm : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                                                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactorTm> shared_ptr;

        DynamicsFactorTm() {}
        DynamicsFactorTm(Key p_i, Key vel_i, Key omega_i, Key input_i, Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model);

        virtual ~DynamicsFactorTm()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector4 &input_i,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;

    private:
        typedef DynamicsFactorTm This;
        typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4,
                                gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
            Base;

        DynamicsParams dynamics_params_;
        
        float dt_;
    };

    /*Trust's x and y components are nozero*/
    class GTSAM_EXPORT DynamicsFactorTm2 : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6, 
        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactorTm2> shared_ptr;

        DynamicsFactorTm2() {}
        DynamicsFactorTm2(Key p_i, Key vel_i, Key omega_i, Key tm_ij, Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model);

        virtual ~DynamicsFactorTm2()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector6 &trust_moments_ij,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;

    private:
        typedef DynamicsFactorTm2 This;
        typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
            Base;

        DynamicsParams dynamics_params_;
        
        float dt_;
    };


    class GTSAM_EXPORT BetForceMoment : public NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<BetForceMoment> shared_ptr;

        BetForceMoment() {}
        BetForceMoment(Key input_i, Key input_j, const SharedNoiseModel &model);

        virtual ~BetForceMoment()
        {
        }

        Vector evaluateError(const gtsam::Vector4 &input_i, const gtsam::Vector4 &input_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none
                             ) const;

    private:
        typedef BetForceMoment This;
        typedef NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>
            Base;
    };

}

#endif // __DYNAMICS_PLANNING_FACTOR_H__