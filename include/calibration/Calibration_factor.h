#ifndef __CALIBRATION_FACTOR_H__
#define __CALIBRATION_FACTOR_H__

#include "gtsam_wrapper.h"
#include "quadrotor_simulator/Dynamics_factor.h"
#include "quadrotor_simulator/Dynamics_params.h"

using namespace gtsam_wrapper;


namespace UAVFactor
{
    /*Scheme 1 : unify Dynamcis modol and actuator model*/
    // Onestage Dynamics model calibration factor
    // Posei Veli Omegai Posej Velj Omegaj inertia_moment, g_Rot, rotor_position, kf, km, body_p_mass
    class GTSAM_EXPORT DynamcisCaliFactor_RS : public NoiseModelFactor13<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3, double, double, gtsam::Pose3, gtsam::Vector3> 
    {        
    public:
        typedef boost::shared_ptr<DynamcisCaliFactor_RS> shared_ptr;

        DynamcisCaliFactor_RS() {}
        DynamcisCaliFactor_RS(Key p_i, Key vel_i, Key omega_i, Key p_j, Key vel_j, Key omega_j, 
            Key im_key, Key rwg_key, Key p_key, Key kf_key, Key km_key, Key bTm_key, Key drag_key, gtsam::Vector4 actuator_outputs, 
            float dt, float mass, const SharedNoiseModel &model);

        virtual ~DynamcisCaliFactor_RS()
        {
        }


        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
                             const gtsam::Vector3 &inertia_moment, const gtsam::Rot3 &rwg, 
                             const gtsam::Vector3 &rotor_pos, const double &ct, const double &km, const gtsam::Pose3& bTm,
                             const gtsam::Vector3 &drag_k,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none, boost::optional<Matrix &> H8 = boost::none,
                             boost::optional<Matrix &> H9 = boost::none, boost::optional<Matrix &> H10 = boost::none,
                             boost::optional<Matrix &> H11 = boost::none, boost::optional<Matrix &> H12 = boost::none,
                             boost::optional<Matrix &> H13 = boost::none) const;

    private: 
        typedef DynamcisCaliFactor_RS This;
        typedef NoiseModelFactor13<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3, double, double, gtsam::Pose3, gtsam::Vector3>
            Base;
        
        float          dt_;
        float          mass_;
        gtsam::Vector4 actuator_outputs_;

        gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class GTSAM_EXPORT DynamcisCaliFactor_RS_AB : public NoiseModelFactor15<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3, double, double, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>
    {
        public:
        typedef boost::shared_ptr<DynamcisCaliFactor_RS> shared_ptr;

        DynamcisCaliFactor_RS_AB() {}
        DynamcisCaliFactor_RS_AB(Key p_i, Key vel_i, Key omega_i, Key p_j, Key vel_j, Key omega_j, 
            Key im_key, Key rwg_key, Key p_key, Key kf_key, Key km_key, Key bTm_key, Key drag_key, Key A_key, Key B_key,
            gtsam::Vector4 actuator_outputs,
            float dt, float mass, const SharedNoiseModel &model);

        virtual ~DynamcisCaliFactor_RS_AB()
        {
        }


        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
                             const gtsam::Vector3 &inertia_moment, const gtsam::Rot3 &rwg, 
                             const gtsam::Vector3 &rotor_pos, const double &ct, const double &km, const gtsam::Pose3& bTm,
                             const gtsam::Vector3 &drag_k, const gtsam::Vector3 &A, const gtsam::Vector3 &B, 
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none, boost::optional<Matrix &> H8 = boost::none,
                             boost::optional<Matrix &> H9 = boost::none, boost::optional<Matrix &> H10 = boost::none,
                             boost::optional<Matrix &> H11 = boost::none, boost::optional<Matrix &> H12 = boost::none,
                             boost::optional<Matrix &> H13 = boost::none, boost::optional<Matrix &> H14 = boost::none,
                             boost::optional<Matrix &> H15 = boost::none) const;
        
        gtsam::Vector6 Thrust_Torque(const gtsam::Vector4 & rpm_square, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos) const;
        gtsam::Vector6 Thrust_Torque(const gtsam::Vector4 & rpm_square, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos, gtsam::Vector3 & A) const;

    private: 
        typedef DynamcisCaliFactor_RS_AB This;
        typedef NoiseModelFactor15<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector3, double, double, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>
            Base;
        
        float          dt_;
        float          mass_;
        gtsam::Vector4 actuator_outputs_;

        gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity
        
        // !!! Check the order of PWM
        const gtsam::Matrix3 rk1_ = gtsam::Vector3( 1,  1, 1).asDiagonal(); 
        const gtsam::Matrix3 rk2_ = gtsam::Vector3( 1, -1, 1).asDiagonal(); 
        const gtsam::Matrix3 rk3_ = gtsam::Vector3(-1, -1, 1).asDiagonal();
        const gtsam::Matrix3 rk4_ = gtsam::Vector3(-1,  1, 1).asDiagonal();
        const gtsam::Vector3 axis = gtsam::Vector3(0, 0, 1);
        const gtsam::Matrix3 axis_mat = gtsam::skewSymmetric(axis);
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };


    /*Motor speed factor*/
    // rpm2 = rpm1 + k * pwm
    class GTSAM_EXPORT RPM_TC_Factor : public NoiseModelFactor3<gtsam::Vector4, gtsam::Vector4, double>
    {
    public:
        typedef boost::shared_ptr<RPM_TC_Factor> shared_ptr;

        RPM_TC_Factor() {}

        RPM_TC_Factor(Key rpm1_key, Key rpm2_key, Key kkey, gtsam::Vector4 & pwm, const SharedNoiseModel &model)
                            : Base(model, rpm1_key, rpm2_key, kkey)
                            , pwm_(pwm){};

        virtual ~RPM_TC_Factor()
        {
        }

        Vector evaluateError(const gtsam::Vector4& rpm1, const gtsam::Vector4& rpm2, const double &k,
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none) const;

    private:
        typedef RPM_TC_Factor This;
        typedef NoiseModelFactor3<gtsam::Vector4, gtsam::Vector4, double>
            Base;
        
        gtsam::Vector4 pwm_; // rotor's PWM

    };



    /*Scheme 2 : Dynamcis modol + actuator model, respectively*/
    // Dynamics model calibration factor, but no actuator model
    // Posei Veli Omegai Thrust_torque Posej Velj Omegaj inertia_moment, g_Rot
    class GTSAM_EXPORT DynamcisCaliFactor_TM : public NoiseModelFactor9<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, 
        gtsam::Vector6, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3>
    {        

    public:
        typedef boost::shared_ptr<DynamcisCaliFactor_TM> shared_ptr;

        DynamcisCaliFactor_TM() {}
        DynamcisCaliFactor_TM(Key p_i, Key vel_i, Key omega_i, Key tm_ij, Key p_j, Key vel_j, Key omega_j, 
            Key im_key, 
            Key rwg_key, 
            float dt, float mass, const SharedNoiseModel &model);

        virtual ~DynamcisCaliFactor_TM()
        {
        }

        /*Compute error and Jacobian: pos_i wTbody_i, bodyTmass*/
        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
                             const gtsam::Vector6 &thrust_torque,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j, 
                             const gtsam::Vector3 &inertia_moment, const gtsam::Rot3 &rwg, 
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none, boost::optional<Matrix &> H8 = boost::none,
                             boost::optional<Matrix &> H9 = boost::none) const;

    private: 
        typedef DynamcisCaliFactor_TM This;
        typedef NoiseModelFactor9<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector6,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3, gtsam::Rot3>
            Base;
        

        float          dt_;
        float          mass_;
        gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity
        
    };

    // Allocation Control Factor <thrust, moments, 
    // rotor_pos of rotor1, rot3 of rotor's axis, ct of thrust model, km of inertia moments model >
    class GTSAM_EXPORT AllocationCalibFactor : public NoiseModelFactor7<gtsam::Vector3, gtsam::Vector3,
        gtsam::Vector3, gtsam::Rot3, double, double, double>
    {
    public:
        typedef boost::shared_ptr<AllocationCalibFactor> shared_ptr;

        AllocationCalibFactor() {}

        AllocationCalibFactor(Key thrust_key, Key moments_key, Key rotor_pos_key, Key rotor_axis_key, 
                            Key ct_key, Key km_key, Key k_esc_key, 
                            ActuatorEffectivenessRotors &actuatorEffectivenessRotors, gtsam::Vector4 & actuators, 
                            const SharedNoiseModel &model)
                            : Base(model, thrust_key, moments_key, rotor_pos_key, rotor_axis_key, ct_key, km_key, k_esc_key)
                            , actuatorEffectivenessRotors_(actuatorEffectivenessRotors)
                            , actuators_(actuators){};

        virtual ~AllocationCalibFactor()
        {
        }

        Vector evaluateError(const gtsam::Vector3& thrust, const gtsam::Vector3 & moments,
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

    };

    // Allocation Control Factor 3 <thrust_moments, rotor_pos of rotor1, ct of thrust model, km of inertia moments model>.
    class GTSAM_EXPORT AllocationCalibFactor3 : public NoiseModelFactor4<gtsam::Vector6, gtsam::Vector3, double, double>
    {
    public:
        typedef boost::shared_ptr<AllocationCalibFactor3> shared_ptr;

        AllocationCalibFactor3() {}

        AllocationCalibFactor3(Key thrust_moments_key, Key rotor_pos_key, Key ct_key, Key km_key, 
                            gtsam::Vector4 & actuators, const SharedNoiseModel &model)
                            : Base(model, thrust_moments_key, rotor_pos_key, ct_key, km_key)
                            , actuators_(actuators){};

        virtual ~AllocationCalibFactor3()
        {
        }

        Vector evaluateError(const gtsam::Vector6& thrust_moments, const gtsam::Vector3& rotor_pos, 
                            const double &ct, const double &km,
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none) const;

    private:
        typedef AllocationCalibFactor3 This;
        typedef NoiseModelFactor4<gtsam::Vector6, gtsam::Vector3, double, double>
            Base;
        
        gtsam::Vector4 actuators_; // rotor's PWM

    };

    class GTSAM_EXPORT MotorCalibFactor: public NoiseModelFactor1<gtsam::Vector5>
    {
        public:
            typedef boost::shared_ptr<MotorCalibFactor> shared_ptr;
            MotorCalibFactor(Key s3_key, double battery_voltage, double pwm, double rotor_speed1, double rotor_speed2, double dt,
                            const SharedNoiseModel &model)
                            : Base(model, s3_key)
                            , battery_voltage_(battery_voltage)
                            , pwm_(pwm)
                            , rotor_speed1_(rotor_speed1)
                            , rotor_speed2_(rotor_speed2)
                            , dt_(dt){};

            virtual ~MotorCalibFactor()
            {   
            }

            Vector evaluateError(const gtsam::Vector5 & params, boost::optional<Matrix &> H1 = boost::none) const;

    private:
        typedef MotorCalibFactor This;
        typedef NoiseModelFactor1<gtsam::Vector5> Base;
        
        double battery_voltage_;
        double pwm_;
        double rotor_speed1_;
        double rotor_speed2_;
        double dt_;
    };

}

#endif // __CALIBRATION_FACTOR_H__