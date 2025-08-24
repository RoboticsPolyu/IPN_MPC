#include "quadrotor_simulator/CBF_factor.h"

namespace UAVFactor
{
   Vector VeCBFPdFactor::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3) const
   {
      Vector err;
      Matrix36 jac_ti;
      // err = (- Vector1(1.* safe_d_ * safe_d_) 
      //      + (pi.translation(jac_t_posei) - obs_).transpose() * (pi.translation(jac_t_posei) - obs_)); 
      gtsam::Vector3 p_p0 = pi.translation(jac_ti) - obs_;
      double d = p_p0.norm();
      gtsam::Vector1 hi = Vector1(1/safe_d_ - 1/d) + beta_* p_p0.transpose()/d* (vi - obs_vel_);
      
      gtsam::Vector3 e2(0., 0., 1.);
      gtsam::Vector3 _g(0., 0., 9.81);
      gtsam::Vector3 _ai = e2 * ui(0);
      
      gtsam::Matrix36 J_ri;
      gtsam::Matrix3  J_r_ri;
      gtsam::Matrix3  J_r_ai;

      gtsam::Vector3 x_dot_0 = vi;
      gtsam::Vector3 x_dot_2 = -_g + pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
      err = hi + alpha_* evaluateH_ti(pi, vi)* x_dot_0 + alpha_* evaluateH_vi(pi)* x_dot_2; // h + alpha* dot h

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
            
            if(H3)
            {
               *H3 = gtsam::Matrix14::Zero();
            }
      }
      else
      {
            if(H1) 
            {
               *H1 = evaluateH_ti_err(pi, vi, ui)* jac_ti 
                  + alpha_* evaluateH_vi(pi)* J_r_ri* J_ri ;
            }

            if(H2)
            {
               *H2 = evaluateH_vi_err(pi, vi);
            }

            if(H3)
            {
               gtsam::Matrix14 h3; h3.setZero();
               gtsam::Matrix13 J_ai = alpha_* evaluateH_vi(pi)* J_r_ai;
               h3.block(0, 0, 0, 0) = J_ai* e2;
               *H3 = h3;
            }
      }

      return err;
   }

   const Vector3 kGravity(0.0, 0.0, 9.81);
   const double kEpsilon = 1e-8;

   Vector VeCBFPdFactorcCylinder::evaluateError(const gtsam::Pose3 &pi, 
                                                const gtsam::Vector3 &vi, 
                                                const gtsam::Vector4 &ui,
                                                boost::optional<Matrix &> H1, 
                                                boost::optional<Matrix &> H2, 
                                                boost::optional<Matrix &> H3) const
   {
      gtsam::Matrix36 jac_ti;
      gtsam::Matrix36 J_ri;
      gtsam::Matrix3  J_r_ri;
      gtsam::Matrix3  J_r_ai; 
      gtsam::Vector3 e2(0., 0., 1.);

      gtsam::Vector3 pi_t = pi.translation(jac_ti);
      gtsam::Vector3 _ai = e2 * ui(0);
      gtsam::Vector3 rA = pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);

      Vector err;
   
      gtsam::Matrix23 e12; 
      e12 << 1,0,0, 0,1,0;
      gtsam::Vector2 p_p0 = e12 * (pi_t - obs_);
      gtsam::Vector2 v_v0 = e12 * (vi - obs_vel_);

  
      double d = p_p0.norm();
      double d2 = d * d;
      double d3 = d2 * d;

      gtsam::Vector1 hi = Vector1(1/safe_d_ - 1/d) + beta_* p_p0.transpose()/d * v_v0;

      // std::cout << "p_p0: " << p_p0.transpose() << " - v_v0: " << v_v0.transpose() << " - hi:  " << hi << std::endl;

      gtsam::Vector3 _g(0., 0., 9.81);
      

      gtsam::Vector3 x_dot_0 = vi;
      gtsam::Vector3 x_dot_2 = -_g + rA;

      // Compute Jacobian with respect to pi_t
      gtsam::Matrix13 J_pi = (1.0/d3 * p_p0.transpose() + 
                           beta_/d * v_v0.transpose() - 
                           beta_*(p_p0.dot(v_v0))/d3 * p_p0.transpose()) * e12;

      // Compute Jacobian with respect to vi  
      gtsam::Matrix13 J_vi = (beta_/d * p_p0.transpose()) * e12;

      err = hi + alpha_* (J_pi * x_dot_0 + J_vi * x_dot_2); // h + alpha* dot h
      
      // Compute second derivatives
      gtsam::Matrix33 dJ_vi_dpi_t = beta_ * (
         -1.0/d3/d2 * (e12.transpose() * p_p0) * (p_p0.transpose() * e12)  // outer product
         + 1.0/d * (e12.transpose() * e12)                                  // identity-like term
      );

      // For dJ_pi_dpi_t, compute each term separately
      gtsam::Matrix33 term1 = -3.0/d3/d2 * (e12.transpose() * p_p0) * (p_p0.transpose() * e12)
                           + 1.0/d3 * (e12.transpose() * e12);

      gtsam::Matrix33 term2 = -beta_/d3 * (e12.transpose() * p_p0) * (v_v0.transpose() * e12);

      double p_dot_v = p_p0.dot(v_v0);
      gtsam::Matrix33 term3 = beta_ * (
         (1.0/d3 * (e12.transpose() * v_v0) - 3.0*p_dot_v/d3/d2 * (e12.transpose() * p_p0)) * (p_p0.transpose() * e12)
         + p_dot_v/d3 * (e12.transpose() * e12)
      );

      gtsam::Matrix33 dJ_pi_dpi_t = term1 + term2 - term3;
      
      gtsam::Matrix13 dJ_vi = dJ_pi_dpi_t * vi;
      gtsam::Matrix13 dJ_xd2 = dJ_vi_dpi_t * x_dot_2;

      // Final Jacobian of err w.r.t pi_t
      gtsam::Matrix13 J_err_pi = J_pi + alpha_ * ( dJ_vi + dJ_xd2);


      // Compute ∂J_pi/∂vi
      gtsam::Matrix33 dJ_pi_dvi = (beta_/d) * (e12.transpose() * e12) 
                              - (beta_/d3) * (e12.transpose() * p_p0) * (p_p0.transpose() * e12);

      // Final Jacobian of err with respect to vi
      gtsam::Matrix13 J_err_vi = J_vi + alpha_ * (gtsam::Matrix13(dJ_pi_dvi * vi) + J_pi);

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
            
            if(H3)
            {
               *H3 = gtsam::Matrix14::Zero();
            }
      }
      else
      {
            if(H1) 
            {
               H1->setZero();
               *H1 = J_err_pi * jac_ti + alpha_*  J_vi * J_r_ri * J_ri;
            }

            if(H2)
            {
               H2->setZero();
               *H2 = J_err_vi;
            }

            if(H3)
            {
               gtsam::Matrix14 h3; h3.setZero();
               gtsam::Matrix13 J_ai = alpha_*  J_vi * J_r_ai;
               h3.block(0, 0, 0, 0) = J_ai* e2;
               *H3 = h3;
            }
      }

      return err;
   }


}