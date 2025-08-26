#ifndef __CBF_FACTOR_H__
#define __CBF_FACTOR_H__

#include "Dynamics_factor.h"
#include "Dynamics_params.h"
#include "gtsam_wrapper.h"

#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor
{    
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
            double d = std::max(p_p0.norm(), 1e-6);

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


    
    // Point distance based CBF
    class GTSAM_EXPORT CBFPdFactorCylinder : public NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>
    {
    public:
        typedef boost::shared_ptr<CBFPdFactorCylinder> shared_ptr;

        CBFPdFactorCylinder() {}
        CBFPdFactorCylinder(Key p_i, Key v_i, gtsam::Vector3& obs, double safe_d, double alpha, const SharedNoiseModel &model)
            : Base(model, p_i, v_i), safe_d_(safe_d), alpha_(alpha)
        {
            _E12 << 1,0,0, 0,1,0, 0,0,0;
            obs_ = _E12 * obs;
        }

        virtual ~CBFPdFactorCylinder()
        {
        }

        Vector evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, 
            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const
        {
            Vector err;
            Matrix36 jac_t_posei;
            gtsam::Vector3 p_p0 = _E12 * pi.translation(jac_t_posei) - obs_;
            
            double d = std::max(p_p0.norm(), 1e-6);
            gtsam::Vector3 vi_proj = _E12 * vi; // 投影速度到xy平面

            // 计算误差: h(x) + α * ∂h/∂x * v
            gtsam::Vector1 h_dot = alpha_ * (p_p0.transpose() * vi_proj) / (d * d * d);
            err = gtsam::Vector1(1/safe_d_ - 1/d) + h_dot;
            
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
                    // 计算各项中间结果
                    H1->setZero();
                    
                    double dot_p_vi = p_p0.transpose() * vi_proj; // p_p0^T * v_i
                    
                    // ∂h/∂pose = (∂h/∂d) * (∂d/∂p_p0) * (∂p_p0/∂pose)
                    // ∂h/∂d = 1/(d²)
                    // ∂d/∂p_p0 = p_p0ᵀ/d
                    // ∂p_p0/∂pose = E₁₂ * ∂t/∂pose
                    
                    gtsam::Matrix16 dh_dpose = (p_p0.transpose() / (d * d * d)) * _E12 * jac_t_posei;
                    
                    // ∂(h_dot)/∂pose = α * ∂[(p_p0ᵀ·v_i)/d³]/∂pose
                    // = α * [ (∂(p_p0ᵀ)/∂pose · v_i)/d³ + p_p0ᵀ·v_i · ∂(1/d³)/∂pose ]
                    // = α * [ (v_iᵀ · ∂p_p0/∂pose)/d³ - 3(p_p0ᵀ·v_i)/(d⁵) · p_p0ᵀ · ∂p_p0/∂pose ]
                    
                    gtsam::Matrix13 viE = vi_proj.transpose() * _E12;
                    gtsam::Matrix13 pp0E = p_p0.transpose() * _E12;
                    gtsam::Matrix16 dh_dot_dpose = alpha_ * (
                        (viE * jac_t_posei) / (d * d * d) -
                        (3 * dot_p_vi / (d * d * d * d * d)) * (pp0E * jac_t_posei)
                    );
                    
                    // 总雅可比: ∂error/∂pose = ∂h/∂pose + ∂(h_dot)/∂pose
                    *H1 = dh_dpose + dh_dot_dpose;
                }

                if(H2)
                {
                    // ∂error/∂v_i = α * p_p0ᵀ/d³ * E₁₂
                    H2->setZero();
                    *H2 = alpha_ * (p_p0.transpose() / (d * d * d)) * _E12;
                }
            }

            return err;
        }

    private:
        typedef CBFPdFactorCylinder This;
        typedef NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> Base;
        gtsam::Vector3 obs_;
        double safe_d_;
        double alpha_;
        gtsam::Matrix3 _E12; 
    };



    // Velocity extended CBF
    class GTSAM_EXPORT VeCBFPdFactor1 : public NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<VeCBFPdFactor1> shared_ptr;

        VeCBFPdFactor1() {}
        VeCBFPdFactor1(Key p_i, Key v_i, Key u_i, const gtsam::Vector3& obs, const gtsam::Vector3& obs_vel, double safe_d, double alpha, double beta, const SharedNoiseModel &model)
            : Base(model, p_i, v_i, u_i), obs_(obs), obs_vel_(obs_vel), safe_d_(safe_d), alpha_(alpha), beta_(beta)
        {
        }

        virtual ~VeCBFPdFactor1() {}

        // Jacobian: hi to ti (grad_p h ^T)
        gtsam::Matrix13 evaluateH_ti(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t, const gtsam::Vector3& v_diff, double p_dot_vdiff) const 
        {
            gtsam::Matrix13 H1 = p_p0_t / (d * d * d);
            gtsam::Matrix13 H2 = beta_ * (v_diff.transpose() / d - p_dot_vdiff * p_p0_t / (d * d * d));
            return H1 + H2;
        }
        
        // Jacobian: hi to vi (grad_v h ^T)
        gtsam::Matrix13 evaluateH_vi(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t) const 
        {
            return (beta_ / d) * p_p0_t;
        }

        // Full Jacobian: eta to ti (including Hessians)
        gtsam::Matrix13 evaluateH_ti_err(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t, const gtsam::Vector3& v_diff, double p_dot_vdiff, 
                                        const gtsam::Vector3& term3_vec, const gtsam::Matrix13& H_ti) const 
        {
            double epsilon = 1e-6;
            double d_eps = std::max(d, epsilon);
            double d3 = d_eps * d_eps * d_eps;
            double d5 = d3 * d_eps * d_eps;

            gtsam::Matrix13 H3 = beta_ * (term3_vec.transpose() / d_eps - p_p0_t.dot(term3_vec) * p_p0_t / d3);

            // Additional Hessian term: alpha * (v_diff^T * Hess h)
            double v_norm_sq = v_diff.squaredNorm();
            double s = p_dot_vdiff;  // r^T v_d
            gtsam::Matrix13 hess_term = alpha_ * (v_diff.transpose() / d3 - 3.0 * s * p_p0_t / d5) +
                                        alpha_ * beta_ * (-v_norm_sq / d3 + 3.0 * s * s / d5) * p_p0_t -
                                        alpha_ * beta_ * 2.0 * (s / d3) * v_diff.transpose();

            return H_ti + alpha_ * H3 + hess_term;
        }

        // Full Jacobian: eta to vi (including cross Hessians)
        gtsam::Matrix13 evaluateH_vi_err(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t, const gtsam::Vector3& v_diff, double p_dot_vdiff, 
                                        const gtsam::Matrix13& H_ti, const gtsam::Matrix13& H_vi) const 
        {
            double epsilon = 1e-6;
            double d_eps = std::max(d, epsilon);
            double d3 = d_eps * d_eps * d_eps;

            // H2 for the additional term
            gtsam::Matrix13 H2 = beta_ * (v_diff.transpose() / d_eps - p_dot_vdiff * p_p0_t / d3);

            return H_vi + alpha_ * H_ti + alpha_ * H2;
        }

        gtsam::Vector evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none, 
                                    boost::optional<gtsam::Matrix &> H3 = boost::none) const;

    private:
        typedef VeCBFPdFactor1 This;
        typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4> Base;
        gtsam::Vector3 obs_;
        gtsam::Vector3 obs_vel_;
        double safe_d_;
        double alpha_ = 0.0, beta_ = 0.0;  // Note: Defaults disable extensions; use constructor params.
    };

    gtsam::Vector VeCBFPdFactor1::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
                                            boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2, 
                                            boost::optional<gtsam::Matrix &> H3) const
    {
        double epsilon = 1e-6;

        gtsam::Matrix36 jac_ti;
        gtsam::Vector3 p_p0 = pi.translation(jac_ti) - obs_;
        double d = p_p0.norm();
        double d_eps = std::max(d, epsilon);
        gtsam::Matrix13 p_p0_t = p_p0.transpose();
        gtsam::Vector3 v_diff = vi - obs_vel_;
        double p_dot_vdiff = p_p0.dot(v_diff);

        gtsam::Matrix13 H_ti = evaluateH_ti(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff);
        gtsam::Matrix13 H_vi = evaluateH_vi(p_p0, d_eps, p_p0_t);

        gtsam::Vector3 _e2(0., 0., 1.);  // Note: Assumes z-axis thrust; ui(1:3) unused.
        gtsam::Vector3 _g(0., 0., 9.81);
        gtsam::Vector3 _ai = _e2 * ui(0);

        gtsam::Matrix36 J_ri;
        gtsam::Matrix3 J_r_ri, J_r_ai;
        gtsam::Vector3 R_ai = pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
        gtsam::Vector3 term3_vec = -_g + R_ai;  // a_i

        gtsam::Vector1 hi = gtsam::Vector1(1.0 / safe_d_ - 1.0 / d_eps) + beta_  / d_eps * p_p0_t * v_diff;
        gtsam::Vector1 x_dot_0_term = H_ti * vi;
        gtsam::Vector1 x_dot_2_term = H_vi * term3_vec;
        gtsam::Vector1 err = hi + alpha_ * x_dot_0_term + alpha_ * x_dot_2_term;

        if (err(0) > 0.0)
        {
            err(0) = 0.0;
            if (H1) *H1 = gtsam::Matrix::Zero(1, 6);
            if (H2) *H2 = gtsam::Matrix::Zero(1, 3);
            if (H3) *H3 = gtsam::Matrix::Zero(1, 4);
        }
        else
        {
            if (H1) 
            {
                gtsam::Matrix13 H_ti_err = evaluateH_ti_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, term3_vec, H_ti);
                *H1 = H_ti_err * jac_ti + alpha_ * H_vi * J_r_ri * J_ri;
            }

            if (H2)
            {
                gtsam::Matrix13 H_vi_err = evaluateH_vi_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, H_ti, H_vi);
                *H2 = H_vi_err;
            }
            
            if (H3)
            {
                gtsam::Matrix14 h3 = gtsam::Matrix14::Zero();
                gtsam::Matrix13 J_ai = alpha_ * H_vi * J_r_ai;
                h3.block<1, 1>(0, 0) = J_ai * _e2;  // Scalar, but matrix form
                *H3 = h3;
            }
        }

        return err;
    }

    

    // Velocity extended CBF Cylinder1
    class GTSAM_EXPORT VeCBFPdFactorCylinder1: public NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<VeCBFPdFactorCylinder1> shared_ptr;

        VeCBFPdFactorCylinder1() {}
        VeCBFPdFactorCylinder1(Key p_i, Key v_i, Key u_i, const gtsam::Vector3& obs, const gtsam::Vector3& obs_vel, double safe_d, double alpha, double beta, const SharedNoiseModel &model)
            : Base(model, p_i, v_i, u_i), safe_d_(safe_d), alpha_(alpha), beta_(beta)
        {
            _E12 << 1, 0, 0, 
                    0, 1, 0, 
                    0, 0, 0;
            
            obs_     = _E12 * obs;
            obs_vel_ = _E12 * obs_vel;
        }

        virtual ~VeCBFPdFactorCylinder1() {}

        // Jacobian: hi to ti (grad_p h ^T)
        gtsam::Matrix13 evaluateH_ti(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t, const gtsam::Vector3& v_diff, double p_dot_vdiff) const 
        {
            gtsam::Matrix13 H1 = p_p0_t / (d * d * d);
            gtsam::Matrix13 H2 = beta_ * (v_diff.transpose() / d - p_dot_vdiff * p_p0_t / (d * d * d));
            return H1 + H2;
        }
        
        // Jacobian: hi to vi (grad_v h ^T)
        gtsam::Matrix13 evaluateH_vi(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t) const 
        {
            return (beta_ / d) * p_p0_t;
        }

        // Full Jacobian: eta to ti (including Hessians)
        gtsam::Matrix13 evaluateH_ti_err(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t, const gtsam::Vector3& v_diff, double p_dot_vdiff, 
                                        const gtsam::Vector3& term3_vec, const gtsam::Matrix13& H_ti) const 
        {
            double epsilon = 1e-6;
            double d_eps = std::max(d, epsilon);
            double d3 = d_eps * d_eps * d_eps;
            double d5 = d3 * d_eps * d_eps;

            gtsam::Matrix13 H3 = beta_ * (term3_vec.transpose() / d_eps - p_p0_t.dot(term3_vec) * p_p0_t / d3);

            // Additional Hessian term: alpha * (v_diff^T * Hess h)
            double v_norm_sq = v_diff.squaredNorm();
            double s = p_dot_vdiff;  // r^T v_d
            gtsam::Matrix13 hess_term = alpha_ * (v_diff.transpose() / d3 - 3.0 * s * p_p0_t / d5) +
                                        alpha_ * beta_ * (-v_norm_sq / d3 + 3.0 * s * s / d5) * p_p0_t -
                                        alpha_ * beta_ * 2.0 * (s / d3) * v_diff.transpose();

            return H_ti + alpha_ * H3 + hess_term;
        }

        // Full Jacobian: eta to vi (including cross Hessians)
        gtsam::Matrix13 evaluateH_vi_err(const gtsam::Vector3& p_p0, double d, const gtsam::Matrix13& p_p0_t, const gtsam::Vector3& v_diff, double p_dot_vdiff, 
                                        const gtsam::Matrix13& H_ti, const gtsam::Matrix13& H_vi) const 
        {
            double epsilon = 1e-6;
            double d_eps = std::max(d, epsilon);
            double d3 = d_eps * d_eps * d_eps;

            // H2 for the additional term
            gtsam::Matrix13 H2 = beta_ * (v_diff.transpose() / d_eps - p_dot_vdiff * p_p0_t / d3);

            return H_vi + alpha_ * H_ti + alpha_ * H2;
        }

        gtsam::Vector evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none, 
                                    boost::optional<gtsam::Matrix &> H3 = boost::none) const;

    private:
        typedef VeCBFPdFactorCylinder1 This;
        typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4> Base;
        gtsam::Vector3 obs_;
        gtsam::Vector3 obs_vel_;
        double safe_d_;
        double alpha_ = 0.0, beta_ = 0.0;  // Note: Defaults disable extensions; use constructor params.

        gtsam::Matrix3 _E12;
    };

    gtsam::Vector VeCBFPdFactorCylinder1::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
                                            boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2, 
                                            boost::optional<gtsam::Matrix &> H3) const
    {
        double epsilon = 1e-6;

        gtsam::Matrix36 jac_ti;
        gtsam::Vector3 p_p0 = (_E12 * pi.translation(jac_ti) - obs_);
        double d = p_p0.norm();
        double d_eps = std::max(d, epsilon);
        gtsam::Matrix13 p_p0_t = p_p0.transpose();
        gtsam::Vector3 _vi = _E12 * vi;
        gtsam::Vector3 v_diff = _vi - obs_vel_;
        double p_dot_vdiff = p_p0.dot(v_diff);

        gtsam::Matrix13 H_ti = evaluateH_ti(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff);
        gtsam::Matrix13 H_vi = evaluateH_vi(p_p0, d_eps, p_p0_t);

        gtsam::Vector3 _e2(0., 0., 1.);  // Note: Assumes z-axis thrust; ui(1:3) unused.
        gtsam::Vector3 _g(0., 0., 9.81);
        gtsam::Vector3 _ai = _e2 * ui(0);

        gtsam::Matrix36 J_ri;
        gtsam::Matrix3 J_r_ri, J_r_ai;
        gtsam::Vector3 R_ai = pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
        gtsam::Vector3 term3_vec = -_g + R_ai;  // a_i

        gtsam::Vector1 hi = gtsam::Vector1(1.0 / safe_d_ - 1.0 / d_eps) + beta_  / d_eps * p_p0_t * v_diff;
        gtsam::Vector1 x_dot_0_term = H_ti * _vi;
        gtsam::Vector1 x_dot_2_term = H_vi * term3_vec;
        gtsam::Vector1 err = hi + alpha_ * x_dot_0_term + alpha_ * x_dot_2_term;

        if (err(0) > 0.0)
        {
            err(0) = 0.0;
            if (H1) *H1 = gtsam::Matrix::Zero(1, 6);
            if (H2) *H2 = gtsam::Matrix::Zero(1, 3);
            if (H3) *H3 = gtsam::Matrix::Zero(1, 4);
        }
        else
        {
            if (H1) 
            {
                gtsam::Matrix13 H_ti_err = evaluateH_ti_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, term3_vec, H_ti);
                *H1 = H_ti_err * _E12 * jac_ti + alpha_ * H_vi * J_r_ri * J_ri;
            }

            if (H2)
            {
                gtsam::Matrix13 H_vi_err = evaluateH_vi_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, H_ti, H_vi) * _E12;
                *H2 = H_vi_err;
            }
            
            if (H3)
            {
                gtsam::Matrix14 h3 = gtsam::Matrix14::Zero();
                gtsam::Matrix13 J_ai = alpha_ * H_vi * J_r_ai;
                h3.block<1, 1>(0, 0) = J_ai * _e2;  // Scalar, but matrix form
                *H3 = h3;
            }
        }

        return err;
    }
    

    
    
    // Deprecated function 
    
    // Veclocity extended CBF
    class GTSAM_EXPORT VeCBFPdFactor : public NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<VeCBFPdFactor> shared_ptr;

        VeCBFPdFactor() {}
        VeCBFPdFactor(Key p_i, Key v_i, Key u_i, gtsam::Vector3& obs, gtsam::Vector3& obs_vel, double safe_d, double alpha, double beta, const SharedNoiseModel &model)
            : Base(model, p_i, v_i, u_i), obs_(obs), obs_vel_(obs_vel), safe_d_(safe_d), alpha_(alpha), beta_(beta)
        {
        }

        virtual ~VeCBFPdFactor()
        {
        }

        // Jacobian: hi to ti
        gtsam::Matrix13 evaluateH_ti(const gtsam::Vector3& pi_t, const gtsam::Vector3 &vi) const 
        {
            gtsam::Vector3 p_p0 = pi_t - obs_;
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
        gtsam::Matrix13 evaluateH_vi(const gtsam::Vector3& pi_t) const 
        {
            gtsam::Vector3 p_p0 = pi_t - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose(); // 1x3
            return (beta_ / d) * p_p0_t;
        }

        gtsam::Matrix13 evaluateH_ti_err(const gtsam::Vector3& pi_t, const gtsam::Rot3& rot, const gtsam::Vector3& vi, const gtsam::Vector4& ui) const 
        {
            gtsam::Matrix13 H_ti = evaluateH_ti(pi_t, vi);
            gtsam::Vector3 p_p0 = pi_t - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose();
            gtsam::Vector3 _g(0., 0., 9.81);
            gtsam::Vector3 _ai(0., 0., ui(0));
            gtsam::Vector3 R_ai = rot.rotate(_ai);
            gtsam::Vector3 term3_vec = -_g + R_ai;
            gtsam::Matrix13 H3 = beta_ * (term3_vec.transpose() / d - p_p0_t.dot(term3_vec) * p_p0_t / (d * d * d));
            return H_ti + alpha_ * H3; // Note: Second derivative term omitted for simplicity
        }

        gtsam::Matrix13 evaluateH_vi_err(const gtsam::Vector3& pi_t, const gtsam::Vector3& vi) const 
        {
            gtsam::Vector3 p_p0 = pi_t - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose();
            gtsam::Matrix13 H_vi = (beta_ / d) * p_p0_t;
            gtsam::Matrix13 H_ti = evaluateH_ti(pi_t, vi);
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


    // Veclocity extended CBF Cylinder 
    class GTSAM_EXPORT VeCBFPdFactorcCylinder : public NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4>
    {
    public:
        typedef boost::shared_ptr<VeCBFPdFactorcCylinder> shared_ptr;

        VeCBFPdFactorcCylinder() {}
        VeCBFPdFactorcCylinder(Key p_i, Key v_i, Key u_i, gtsam::Vector3& obs, gtsam::Vector3& obs_vel, double safe_d, double alpha, double beta, const SharedNoiseModel &model)
            : Base(model, p_i, v_i, u_i), obs_vel_(obs_vel), safe_d_(safe_d), alpha_(alpha), beta_(beta)
        {
            _E12 << 1, 0, 0, 
                    0, 1, 0, 
                    0, 0, 0;
            
            obs_ = _E12 * obs;
        }

        virtual ~VeCBFPdFactorcCylinder()
        {
        }

        // Jacobian: hi to ti
        gtsam::Matrix13 evaluateH_ti(const gtsam::Vector3& pi_t, const gtsam::Vector3 &vi) const 
        {
            gtsam::Vector3 p_p0 = pi_t - obs_;
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
        gtsam::Matrix13 evaluateH_vi(const gtsam::Vector3& pi_t) const 
        {
            gtsam::Vector3 p_p0 = pi_t - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose(); // 1x3
            return (beta_ / d) * p_p0_t;
        }

        gtsam::Matrix13 evaluateH_ti_err(const gtsam::Vector3& pi_t, const gtsam::Rot3& rot, gtsam::Vector3& vi, const gtsam::Vector4& ui) const 
        {
            gtsam::Matrix13 H_ti = evaluateH_ti(pi_t, vi);
            gtsam::Vector3 p_p0 = pi_t - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose();
            gtsam::Vector3 _g(0., 0., 9.81);
            gtsam::Vector3 _ai(0., 0., ui(0));

            gtsam::Matrix3 J_r_ri, J_r_ai;
            gtsam::Vector3 R_ai = rot.rotate(_ai, J_r_ri, J_r_ai);
            gtsam::Vector3 term3_vec = -_g + R_ai;
            gtsam::Matrix13 H3 = beta_ * (term3_vec.transpose() / d - p_p0_t.dot(term3_vec) * p_p0_t / (d * d * d));
            return H_ti + alpha_ * H3; // Note: Second derivative term omitted for simplicity
        }

        gtsam::Matrix13 evaluateH_vi_err(const gtsam::Vector3& pi_t, const gtsam::Vector3& vi) const 
        {
            gtsam::Vector3 p_p0 = pi_t - obs_;
            double d = p_p0.norm();
            gtsam::Matrix13 p_p0_t = p_p0.transpose();
            gtsam::Matrix13 H_vi = (beta_ / d) * p_p0_t;
            gtsam::Matrix13 H_ti = evaluateH_ti(pi_t, vi);
            return H_vi + alpha_ * H_ti;
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
        gtsam::Vector3 obs_;
        gtsam::Vector3 obs_vel_;
        double safe_d_;
        double alpha_ = 0.0, beta_ = 0.0;

        gtsam::Matrix3 _E12;
    };

}

#endif