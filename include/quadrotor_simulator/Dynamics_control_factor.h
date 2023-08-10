#ifndef __DYNAMICS_PLANNING_FACTOR_H__
#define __DYNAMICS_PLANNING_FACTOR_H__

#include "Dynamics_factor.h"
#include "Dynamics_params.h"
#include "gtsam_wrapper.h"
#include "Quadrotor_SO3.h"

#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor
{    
    using UAV_State = QuadrotorSim_SO3::Quadrotor::State;


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


    /* position velocity rotation angular_velocity control_input */
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


    /* position velocity rotation angular_velocity control_input:Force and Moment */
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


    /*thrust's x and y components are nozero */
    class GTSAM_EXPORT DynamicsFactorFullTM : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6, 
        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactorFullTM> shared_ptr;

        DynamicsFactorFullTM() {}
        DynamicsFactorFullTM(Key p_i, Key vel_i, Key omega_i, Key tm_ij, Key p_j, Key vel_j, Key omega_j, float dt, const SharedNoiseModel &model);

        virtual ~DynamicsFactorFullTM()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector6 &thrust_torque,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;

    private:
        typedef DynamicsFactorFullTM This;
        typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>
            Base;

        DynamicsParams dynamics_params_;
        
        float dt_;
    };


    /* Force and Moments Between factor */
    class GTSAM_EXPORT BetForceMoments : public NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<BetForceMoments> shared_ptr;

        BetForceMoments() {}
        BetForceMoments(Key input_i, Key input_j, const SharedNoiseModel &model);

        virtual ~BetForceMoments()
        {
        }

        Vector evaluateError(const gtsam::Vector4 &input_i, const gtsam::Vector4 &input_j,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none
                             ) const;

    private:
        typedef BetForceMoments This;
        typedef NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>
            Base;
    };


    class GTSAM_EXPORT ControlLimitFactor : public NoiseModelFactor1<gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<ControlLimitFactor> shared_ptr;

        ControlLimitFactor() {}
        ControlLimitFactor(Key input, const SharedNoiseModel &model, double low, double high, double thr, double alpha)
            : Base(model, input)
            , high_(high)
            , low_(low)
            , thr_(thr)
            , alpha_(alpha){};

        virtual ~ControlLimitFactor()
        {
        }

        Vector evaluateError(const gtsam::Vector4 &input, boost::optional<Matrix &> H1 = boost::none) const;

    private:
        typedef ControlLimitFactor This;
        typedef NoiseModelFactor1<gtsam::Vector4>
            Base;

        double high_;
        double low_;
        double thr_;
        double alpha_;
    };

}

#endif // __DYNAMICS_PLANNING_FACTOR_H__