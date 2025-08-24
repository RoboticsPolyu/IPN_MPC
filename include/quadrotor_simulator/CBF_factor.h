#ifndef __CBF_FACTOR_H__
#define __CBF_FACTOR_H__

#include "Dynamics_factor.h"
#include "Dynamics_params.h"
#include "gtsam_wrapper.h"
#include "Quadrotor_SO3.h"

#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor
{    
    using UAV_State = State;

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
                    *H1 = (p_p0.transpose() * jac_t_posei
                        + alpha_ * (vi.transpose() * jac_t_posei 
                        - 3 * dot_p_vi / (d * d) * p_p0.transpose() * jac_t_posei)) / (d * d * d);
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


    // Veclocity extended CBF VeCBFPdFactorcCylinder
    class GTSAM_EXPORT VeCBFPdFactorcCylinder : public NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<VeCBFPdFactorcCylinder> shared_ptr;

        VeCBFPdFactorcCylinder() {}
        VeCBFPdFactorcCylinder(Key p_i, Key v_i, Key u_i, gtsam::Vector3& obs, gtsam::Vector3& obs_vel, double safe_d, double alpha, double beta, const SharedNoiseModel &model)
            : Base(model, p_i, v_i, u_i), obs_(obs), obs_vel_(obs_vel), safe_d_(safe_d), alpha_(alpha), beta_(beta)
        {}

        virtual ~VeCBFPdFactorcCylinder()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pi, 
                            const gtsam::Vector3 &vi, 
                            const gtsam::Vector4 &ui,
                            boost::optional<Matrix &> H1 = boost::none,
                            boost::optional<Matrix &> H2 = boost::none, 
                            boost::optional<Matrix &> H3 = boost::none) const;

    private:
        typedef VeCBFPdFactorcCylinder This;
        typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4> Base;
        
        double alpha_ = 0.0, beta_ = 0.0;
        gtsam::Vector3 obs_;
        gtsam::Vector3 obs_vel_;
        double safe_d_;
    };

}

#endif