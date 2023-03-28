#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__

#include <boost/array.hpp>
#include <Eigen/Core>
#include "gtsam_wrapper.h"
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>
#include <vector>

namespace QuadrotorSimulator_SO3
{
    class Quadrotor
    {
    public:
        struct State
        {
            Eigen::Vector3d x;
            Eigen::Vector3d v;
            gtsam::Rot3 rot;
            Eigen::Vector3d omega;
            Eigen::Array4d motor_rpm;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };
        
        /********************************* Display *********************************/
        void setup();
        
        void render();

        void pQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot);

        void pCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot);

        void pLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end);

        void pFrame(gtsam::Vector3 p, gtsam::Rot3 rot);
        
       /********************************** Dynamics **********************************/
        Quadrotor();

        const Quadrotor::State &getState(void) const;

        void setState(const Quadrotor::State &state);

        void setStatePos(const Eigen::Vector3d &Pos);

        double getMass(void) const;
        void setMass(double mass);

        double getGravity(void) const;
        void setGravity(double g);

        const Eigen::Matrix3d &getInertia(void) const;
        void setInertia(const Eigen::Matrix3d &inertia);

        double getArmLength(void) const;
        void setArmLength(double d);

        double getPropRadius(void) const;
        void setPropRadius(double r);

        double getPropellerThrustCoefficient(void) const;
        void setPropellerThrustCoefficient(double kf);

        double getPropellerMomentCoefficient(void) const;
        void setPropellerMomentCoefficient(double km);

        double getMotorTimeConstant(void) const;
        void setMotorTimeConstant(double k);

        const Eigen::Vector3d &getExternalForce(void) const;
        void setExternalForce(const Eigen::Vector3d &force);

        const Eigen::Vector3d &getExternalMoment(void) const;
        void setExternalMoment(const Eigen::Vector3d &moment);

        double getMaxRPM(void) const;
        void setMaxRPM(double max_rpm);

        double getMinRPM(void) const;
        void setMinRPM(double min_rpm);

        // -T1 + T2 - T3 + T4
        // -T1 + T2 + T3 - T4
        // -T1 - T2 + T3 + T4
        // Inputs are desired RPM for the motors
        // Rotor numbering is:
        //   *1*    Front
        // 3     4
        //    2
        // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
        void setInput(double u1, double u2, double u3, double u4);

        // Runs the actual dynamics simulation with a time step of dt
        void step(double dt);

        Eigen::Vector3d getAcc() const;

    private:

        double g_; // gravity
        double mass_;
        Eigen::Matrix3d J_; // Inertia
        double kf_;
        double km_;
        double prop_radius_;
        double arm_length_;
        double motor_time_constant_; // unit: sec
        double max_rpm_;
        double min_rpm_;

        Quadrotor::State state_, last_state_;
        std::vector<Quadrotor::State> trj_;

        Eigen::Vector3d acc_;

        Eigen::Array4d input_;
        Eigen::Vector3d external_force_;
        Eigen::Vector3d external_moment_;
        
        std::shared_ptr<pangolin::OpenGlRenderState> s_cam;
        pangolin::View d_cam;
        float axis_dist_;
        float propeller_dist_;
        // const std::string window_name = "HelloPangolinThreads";
    };
}
#endif
