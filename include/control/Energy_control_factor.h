#ifndef __ENERGY_CONTROL_FACTOR_H__
#define __ENERGY_CONTROL_FACTOR_H__

#include "gtsam_wrapper.h"

#include "control/Dynamics_factor.h"
#include "control/Dynamics_params.h"

#include <vector>

class GTSAM_EXPORT EnergyFactor : public NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, float>
{
    public:
    typedef boost::shared_ptr<EnergyFactor> shared_ptr;

    EnergyFactor() {}
    EnergyFactor(Key p_i, Key vel_i, Key input_i, Key dt_i, gtsam::Vector3 drag_k, 
      gtsam::Vector3 model_params, const SharedNoiseModel &model);

    virtual ~EnergyFactor()
    {
    }

    Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector4 &input_i, double dt,
                          boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                          boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none) const;
    
private: 
    typedef EnergyFactor This;
    typedef NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, float>
        Base;

    gtsam::Vector3 drag_k_;
    gtsam::Vector3 model_params_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif // __ENERGY_CONTROL_FACTOR_H__