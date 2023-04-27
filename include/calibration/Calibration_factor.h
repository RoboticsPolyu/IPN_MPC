#ifndef __CALIBRATION_FACTOR_H__
#define __CALIBRATION_FACTOR_H__

#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_factor.h"
#include "quadrotor_simulator/Dynamics_params.h"

using namespace gtsam_wrapper;


namespace UAVFactor
{

    // DynamicsCaliFactorfm: state_i, state_j, trust and moments, moments of inertia 

    class GTSAM_EXPORT DynamcisCaliFactorTrustMoments : public NoiseModelFactor9<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3>
    {        
        typedef DynamcisCaliFactorTrustMoments This;
        typedef NoiseModelFactor9<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
                                gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3>
            Base;
        DynamcisCaliFactorTrustMoments() {}

    public:
        typedef boost::shared_ptr<DynamcisCaliFactorTrustMoments> shared_ptr;

        
        DynamcisCaliFactorTrustMoments(Key p_i, Key vel_i, Key omega_i, Key tm_ij, Key p_j, Key vel_j, Key omega_j, Key im_key, Key rwg_key, 
            float dt, float mass, const SharedNoiseModel &model);

        virtual ~DynamcisCaliFactorTrustMoments()
        {
        }

        /*Compute error and Jacobian: pos_i wTbody_i, bodyTmass*/
        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, const gtsam::Vector6 &trust_moments_ij,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
                             const gtsam::Vector3 &inertia_moments, const gtsam::Rot3 &rwg, 
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none, boost::optional<Matrix &> H8 = boost::none,
                             boost::optional<Matrix &> H9 = boost::none) const;

    private: 
        float dt_;
        gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity
        float mass_;
    };

    // Allocation Control Factor <trust, moments, 
    // rotor_pos of rotor1, rot3 of rotor's axis, ct of trust model, km of inertia moments model, factor of esc model >
    class GTSAM_EXPORT AllocationCalibFactor : public NoiseModelFactor7<gtsam::Vector3, gtsam::Vector3,
        gtsam::Vector3, gtsam::Rot3, double, double, double>
    {
    public:
        typedef boost::shared_ptr<AllocationCalibFactor> shared_ptr;

        AllocationCalibFactor() {}

        AllocationCalibFactor(Key trust_key, Key moments_key, Key rotor_pos_key, Key rotor_axis_key, 
                            Key ct_key, Key km_key, Key k_esc_key, 
                            ActuatorEffectivenessRotors &actuatorEffectivenessRotors, gtsam::Vector4 & actuators, 
                            double PWM_MIN, double PWM_MAX, const SharedNoiseModel &model)
                            : Base(model, trust_key, moments_key, rotor_pos_key, rotor_axis_key, ct_key, km_key, k_esc_key)
                            , actuatorEffectivenessRotors_(actuatorEffectivenessRotors)
                            , actuators_(actuators)
                            , PWM_MIN_(PWM_MIN)
                            , PWM_MAX_(PWM_MAX) {};

        virtual ~AllocationCalibFactor()
        {
        }

        Vector evaluateError(const gtsam::Vector3& trust, const gtsam::Vector3 & moments,
                             const gtsam::Vector3& rotor_pos, const gtsam::Rot3& rotor_axis, 
                             const double &ct, const double &km, const double &esc_factor,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;

    private:
        typedef AllocationCalibFactor This;
        typedef NoiseModelFactor7<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, double, double, double>
            Base;

        ActuatorEffectivenessRotors actuatorEffectivenessRotors_;
        
        gtsam::Vector4 actuators_; // rotor's PWM

        double PWM_MIN_, PWM_MAX_;

    };

    class GTSAM_EXPORT AllocationCalibFactor2 : public NoiseModelFactor6<gtsam::Vector3, gtsam::Vector3,
        gtsam::Vector3, double, double, double>
    {
    public:
        typedef boost::shared_ptr<AllocationCalibFactor2> shared_ptr;

        AllocationCalibFactor2() {}

        AllocationCalibFactor2(Key trust_key, Key moments_key, Key rotor_pos_key, Key ct_key, Key km_key, Key k_esc_key, 
                            gtsam::Vector4 & actuators, double pwm_min, double pwm_max, const SharedNoiseModel &model)
                            : Base(model, trust_key, moments_key, rotor_pos_key, ct_key, km_key, k_esc_key)
                            , actuators_(actuators)
                            , PWM_MIN_(pwm_min)
                            , PWM_MAX_(pwm_max) {};

        virtual ~AllocationCalibFactor2()
        {
        }

        Vector evaluateError(const gtsam::Vector3& trust, const gtsam::Vector3 & moments, const gtsam::Vector3& rotor_pos, 
                            const double &ct, const double &km, const double &esc_factor,
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                            boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none) const;

    private:
        typedef AllocationCalibFactor2 This;
        typedef NoiseModelFactor6<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, double, double, double>
            Base;
        
        gtsam::Vector4 actuators_; // rotor's PWM
        double PWM_MIN_, PWM_MAX_;

    };

    
}

#endif // __CALIBRATION_FACTOR_H__