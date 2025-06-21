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
    using UAV_State = State;


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


    class GTSAM_EXPORT DynamicFactor : public NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, 
        gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4>
    {
        public:
        typedef boost::shared_ptr<DynamicFactor> shared_ptr;

        DynamicFactor() {}
        DynamicFactor(Key p_i, Key vel_i, Key omega_i, Key p_j, Key vel_j, Key omega_j, Key input_i,
            float dt, float mass, gtsam::Vector3 inertia, gtsam::Vector3 rotor_pos, gtsam::Vector3 drag_k, 
            double ctt, double kmm, const SharedNoiseModel &model);

        virtual ~DynamicFactor()
        {
        }


        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector3 &omega_i, 
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Vector3 &omega_j,
                             const gtsam::Vector4 &input_i,
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none,
                             boost::optional<Matrix &> H7 = boost::none) const;
        
        gtsam::Vector6 Thrust_Torque(const gtsam::Vector4 & rpm, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos, gtsam::Matrix64 & Jac) const;
        gtsam::Vector6 Thrust_Torque(const gtsam::Vector4 & rpm_square, const double & ct, const double & km, const gtsam::Vector3 & rotor_pos, gtsam::Vector3 & A) const;

    private: 
        typedef DynamicFactor This;
        typedef NoiseModelFactor7<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
            gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector4>
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
        
        gtsam::Vector3 rot_inertia_, rotor_pos_, drag_k_;
        double ct, km;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

    /*
    * MPC based FGO, generating Thrust and gyro
    */
    class GTSAM_EXPORT DynamicsFactorTGyro : public NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<DynamicsFactorTGyro> shared_ptr;

        DynamicsFactorTGyro() {}
        DynamicsFactorTGyro(Key p_i, Key vel_i, Key input_i, Key p_j, Key vel_j, float dt, double mass_, gtsam::Vector3 drag_k, const SharedNoiseModel &model);

        virtual ~DynamicsFactorTGyro()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector4 &input_i,
                             const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, 
                             boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                             boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                             boost::optional<Matrix &> H5 = boost::none) const;

    private:
        typedef DynamicsFactorTGyro This;
        typedef NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3, gtsam::Vector3>
            Base;

        DynamicsParams dynamics_params_;
        
        gtsam::Vector3 drag_k_;
        
        double mass_;
        
        gtsam::Vector3 gI_ = gtsam::Vector3(0, 0, 9.81); // gravity

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

    /* Force and Moments Between factor */
    class GTSAM_EXPORT PointObsFactor : public NoiseModelFactor1<gtsam::Pose3>
    {
    public:
        typedef boost::shared_ptr<PointObsFactor> shared_ptr;

        PointObsFactor() {}
        PointObsFactor(Key p_i, gtsam::Vector3& obs, float safe_d, const SharedNoiseModel &model)
            : Base(model, p_i), obs_(obs), safe_d_(safe_d)
        {}

        virtual ~PointObsFactor()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pi, boost::optional<Matrix &> H1 = boost::none) const
        {
            Vector err;
            Matrix36 jac_t_posei;
            double d = (pi.translation(jac_t_posei) - obs_).norm();

            err = Vector1(1/d - 1/safe_d_/1.1);

            if(err(0) < 0)
            {
                err(0) = 0;
                if(H1)
                {
                    *H1 = gtsam::Matrix16::Zero();
                }
            }
            else
            {
                // std::cout << " -- PointObs: [ " << err[0] << " ]" << std::endl;
                if(H1)
                {
                    *H1 = - (pi.translation() - obs_).transpose() * jac_t_posei /d /d /d;
                }
            }

            return err;
        }

    private:
        typedef PointObsFactor This;
        typedef NoiseModelFactor1<gtsam::Pose3>
            Base;
        gtsam::Vector3 obs_;
        float safe_d_;
    };

    // Point distance based CBF
    class GTSAM_EXPORT CBFPdFactor : public NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<CBFPdFactor> shared_ptr;

        CBFPdFactor() {}
        CBFPdFactor(Key p_i, Key v_i, gtsam::Vector3& obs, double safe_d, double alpha, const SharedNoiseModel &model)
            : Base(model, p_i, v_i), obs_(obs), safe_d_(safe_d), alpha_(alpha)
        {}

        virtual ~CBFPdFactor()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, 
            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const
        {
            Vector err;
            Matrix36 jac_t_posei;
            gtsam::Vector3 p_p0 = pi.translation(jac_t_posei) - obs_;
            double d = p_p0.norm();

            err = Vector1(1/safe_d_ - 1/d) + alpha_* p_p0.transpose()/d/d/d* vi; // h + alpha* dot h
            if(err(0) > 0)
            {
                err(0) = 0;
                if(H1)
                {
                    *H1 = gtsam::Matrix16::Zero();
                }

                if(H2)
                {
                    *H2 = gtsam::Matrix13::Zero();
                }
            }
            else
            {
                
                if(H1) 
                {
                    double dot_p_vi = p_p0.transpose() * vi; // p_p0^T * vi
                    *H1 = (p_p0.transpose() * jac_t_posei + 
                        alpha_ * (vi.transpose() * jac_t_posei - 3 * dot_p_vi / (d * d) * p_p0.transpose() * jac_t_posei)) / (d * d * d);
                }

                if(H2)
                {
                    *H2 = alpha_* p_p0.transpose()/d/d/d;
                }
            }

            return err;
        }

    private:
        typedef CBFPdFactor This;
        typedef NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> Base;
        gtsam::Vector3 obs_;
        double safe_d_;
        double alpha_;
    };

    // Veclocity extended CBF
    class GTSAM_EXPORT VeCBFPdFactor : public NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<VeCBFPdFactor> shared_ptr;

        VeCBFPdFactor() {}
        VeCBFPdFactor(Key p_i, Key v_i, Key u_i, gtsam::Vector3& obs, gtsam::Vector3& obs_vel, double safe_d, double alpha, double beta, const SharedNoiseModel &model)
            : Base(model, p_i, v_i, u_i), obs_(obs), obs_vel_(obs_vel), safe_d_(safe_d), alpha_(alpha), beta_(beta)
        {}

        virtual ~VeCBFPdFactor()
        {
        }

        // Jacobian: hi to ti
        gtsam::Matrix13 evaluateH_ti(const gtsam::Pose3& pi, const gtsam::Vector3 &vi) const 
        {
            gtsam::Vector3 p_p0 = pi.translation() - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose(); // 1x3
            gtsam::Matrix13 H1 = p_p0_t / (d * d * d);

            gtsam::Vector3 v_diff = vi - obs_vel_; // vi - v_obs
            gtsam::Matrix13 v_diff_t = v_diff.transpose(); // 1x3
            double p_dot_vdiff = p_p0_t.dot(v_diff);
            gtsam::Matrix13 H2 = beta_ * (v_diff_t / d - p_dot_vdiff * p_p0_t / (d * d * d));

            return H1 + H2;
        }
        
        // Jacobian: hi to vi
        gtsam::Matrix13 evaluateH_vi(const gtsam::Pose3& pi) const 
        {
            gtsam::Vector3 p_p0 = pi.translation() - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose(); // 1x3
            return (beta_ / d) * p_p0_t;
        }

        gtsam::Matrix13 evaluateH_ti_err(const gtsam::Pose3& pi, const gtsam::Vector3& vi, const gtsam::Vector4& ui) const 
        {
            gtsam::Matrix13 H_ti = evaluateH_ti(pi, vi);
            gtsam::Vector3 p_p0 = pi.translation() - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose();
            gtsam::Vector3 _g(0., 0., 9.81);
            gtsam::Vector3 _ai(0., 0., ui(0));
            gtsam::Matrix36 J_ri;
            gtsam::Matrix3 J_r_ri, J_r_ai;
            gtsam::Vector3 R_ai = pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
            gtsam::Vector3 term3_vec = -_g + R_ai;
            gtsam::Matrix13 H3 = beta_ * (term3_vec.transpose() / d - p_p0_t.dot(term3_vec) * p_p0_t / (d * d * d));
            return H_ti + alpha_ * H3; // Note: Second derivative term omitted for simplicity
        }

        gtsam::Matrix13 evaluateH_vi_err(const gtsam::Pose3& pi, const gtsam::Vector3& vi) const 
        {
            gtsam::Vector3 p_p0 = pi.translation() - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose();
            gtsam::Matrix13 H_vi = (beta_ / d) * p_p0_t;
            gtsam::Matrix13 H_ti = evaluateH_ti(pi, vi);
            return H_vi + alpha_ * H_ti;
        }

        Vector evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none, boost::optional<Matrix &> H3 = boost::none) const;

    private:
        typedef VeCBFPdFactor This;
        typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4> Base;
        gtsam::Vector3 obs_;
        gtsam::Vector3 obs_vel_;
        double safe_d_;
        double alpha_ = 0.0, beta_ = 0.0;
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

    class GTSAM_EXPORT ControlLimitTGyroFactor : public NoiseModelFactor1<gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<ControlLimitTGyroFactor> shared_ptr;

        ControlLimitTGyroFactor() {}
        ControlLimitTGyroFactor(Key input, const SharedNoiseModel &model, 
            double T_low, double T_high, double Gyro_low, double Gyro_high,
            double T_thr, double Gyro_thr, double alpha)
            : Base(model, input)
            , T_high_(T_high)
            , T_low_(T_low)
            , Gyro_high_(Gyro_high)
            , Gyro_low_(Gyro_low)
            , T_thr_(T_thr)
            , Gyro_thr_(Gyro_thr)
            , alpha_(alpha){};

        virtual ~ControlLimitTGyroFactor()
        {
        }

        Vector evaluateError(const gtsam::Vector4 &input, boost::optional<Matrix &> H1 = boost::none) const;

    private:
        typedef ControlLimitTGyroFactor This;
        typedef NoiseModelFactor1<gtsam::Vector4>
            Base;

        double T_high_;
        double T_low_;

        double Gyro_high_;
        double Gyro_low_;

        double T_thr_;
        double Gyro_thr_;

        double alpha_;
    };

}

#endif // __DYNAMICS_PLANNING_FACTOR_H__