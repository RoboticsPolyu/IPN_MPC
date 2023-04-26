#ifndef __CALIBRATION_FACTOR_H__
#define __CALIBRATION_FACTOR_H__

#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_params.h"

using namespace gtsam_wrapper;

namespace UAVFactor
{

    // DynamicsCaliFactorfm: state_i, state_j, moments of inertia

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



    
}

#endif // __CALIBRATION_FACTOR_H__