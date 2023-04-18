#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__

#include "color.h"
#include "env_sensors_sim/Common.h"
#include "gtsam_wrapper.h"

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
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

using namespace std;
using namespace boost::numeric::odeint;

namespace QuadrotorSim_SO3
{
    class Quadrotor
    {
    public:
        typedef boost::array<double, 22> stateType;
        struct State
        {
            int64_t id;
            double timestamp;
            Eigen::Vector3d x;
            Eigen::Vector3d v;
            gtsam::Rot3 rot;
            Eigen::Vector3d omega;
            Eigen::Array4d motor_rpm;
            Eigen::Vector4d force_moment;
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };
        
        /********************************* Display *********************************/
        void display_setup();
        
        void render_history_trj();
        
        void render_history_opt(std::vector<State> & trj, gtsam::Vector3 & err, boost::optional<Features&> features = boost::none);

        void pQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot);

        void pCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot);

        void pLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end);

        void pFrame(gtsam::Vector3 p, gtsam::Rot3 rot);
        
        void render_panel();

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
        
        void setInput(gtsam::Vector4 force_moment);

        // Runs the actual dynamics simulation with a time step of dt
        void step(double dt);

        void step_noise(double dt);
        
        void operator()(const Quadrotor::stateType &x , Quadrotor::stateType &dxdt , const double t);

        void stepODE(double dt, gtsam::Vector4 fm);
        void stepODE2(double dt, gtsam::Vector4 fm);

        void printCurState();

        void step_fm(double dt, gtsam::Vector4 fm);

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
        Eigen::Vector3d drag_force_p_;

        Quadrotor::State state_, last_state_;
        std::vector<Quadrotor::State> trj_;

        Eigen::Vector3d acc_;

        Eigen::Array4d input_;
        Eigen::Vector3d external_force_;
        Eigen::Vector3d external_moment_;
        gtsam::Vector4 force_moment_;

        pangolin::View d_cam;
        std::shared_ptr<pangolin::OpenGlRenderState> s_cam;
        std::shared_ptr<pangolin::Var<std::string> > dis_force_;
        std::shared_ptr<pangolin::Var<std::string> > dis_M1_;
        std::shared_ptr<pangolin::Var<std::string> > dis_M2_;
        std::shared_ptr<pangolin::Var<std::string> > dis_M3_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAVx_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAVy_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAVz_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAV_velx_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAV_vely_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAV_velz_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAVRx_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAVRy_;
        std::shared_ptr<pangolin::Var<std::string> > dis_UAVRz_;
        std::shared_ptr<pangolin::Var<std::string> > dis_AVE_ERR_;
        std::shared_ptr<pangolin::Var<std::string> > dis_timestamp_;

        std::vector<gtsam::Vector3> errs_;
        const uint64_t ERRS_LENS = 1;

        float axis_dist_;
        float propeller_dist_;
        const uint64_t HISTORY_TRJ_LENS = 1000;

        std::default_random_engine generator_;
        std::ofstream err_file_;

        // force noise
        double AT_NOISE_MEAN = 0.0;
        double AT_NOISE_COV  = 0.0;
        double ANGULAR_SPEED_MEAN = 0.0;
        double ANGULAR_SPEED_COV = 0.0;
    };
}
#endif
