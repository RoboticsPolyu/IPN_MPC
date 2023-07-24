#ifndef __DYNAMICS_FACTOR_H__
#define __DYNAMICS_FACTOR_H__

#include "Dynamics_params.h"
#include "gtsam_wrapper.h"

#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor
{
    static constexpr int NUM_ACTUATORS = 16;
    static constexpr int NUM_AXES = 6;

    struct Rotor
    {
        double         thrust_coef;
        double         moment_ratio;
        gtsam::Vector3 axis;
        gtsam::Rot3    rot_z;
        gtsam::Vector3 position;
    };

    struct Geometry
    {
        uint8_t num_rotors = 0u;
        Rotor rotors[NUM_ACTUATORS];

        bool propeller_torque_disabled = false;
        bool yaw_by_differential_thrust_disabled = false;
        bool propeller_torque_disabled_non_upwards = false;
        bool three_dimensional_thrust_disabled = false;
    };

    using ActuatorEnabled     = Eigen::Matrix<bool, NUM_ACTUATORS, 1>;
    using EffectivenessMatrix = Eigen::Matrix<float, NUM_AXES, NUM_ACTUATORS>;
    
    class ActuatorEffectivenessRotors
    {
    public:
        ActuatorEffectivenessRotors() = default;
        virtual ~ActuatorEffectivenessRotors() = default;

        ActuatorEffectivenessRotors(ActuatorEnabled &actuatorEnabled, Geometry &geometry)
            : _actuatorEnabled(actuatorEnabled)
            , _geometry(geometry){};
        

        EffectivenessMatrix getEffectivenessMatrix();
        
        int computeEffectivenessMatrix(const Geometry &geometry, EffectivenessMatrix &effectiveness, 
                                    ActuatorEnabled &actuatorEnabled, boost::optional<gtsam::Matrix &> Jacobian);

    private:
        

        Geometry _geometry;
        ActuatorEnabled _actuatorEnabled;
    };


    // Allocation Control Factor <thrust, moments, actuators_output>
    class GTSAM_EXPORT AllocationFactor : public NoiseModelFactor3<double, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<AllocationFactor> shared_ptr;

        AllocationFactor() {}

        AllocationFactor(Key thrust_key, Key moments_key, Key actuators_key,
                         ActuatorEffectivenessRotors &actuatorEffectivenessRotors, const SharedNoiseModel &model)
                         : Base(model, thrust_key, moments_key, actuators_key),
                         actuatorEffectivenessRotors_(actuatorEffectivenessRotors)
                         {};

        virtual ~AllocationFactor()
        {
        }

        Vector evaluateError(const double& thrust, const gtsam::Vector3 & moments, const gtsam::Vector4 & actuator_outputs,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none) const;

    private:
        typedef AllocationFactor This;
        typedef NoiseModelFactor3<double, gtsam::Vector3, gtsam::Vector4>
            Base;

        ActuatorEffectivenessRotors actuatorEffectivenessRotors_;

    };

    
}

#endif // __DYNAMICS_FACTOR_H__