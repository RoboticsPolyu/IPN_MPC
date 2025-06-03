#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__

#include "color.h"
#include "Dynamics_factor.h"
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


using namespace boost::numeric::odeint;
using namespace std;
using namespace UAVFactor;


namespace QuadrotorSim_SO3
{
    class Quadrotor
    {
    public:
        using stringUI = std::shared_ptr<pangolin::Var<std::string> >;
        using stateType = boost::array<double, 22>;

        struct State
        {
            int64_t         id;
            double          timestamp;

            Eigen::Vector3d p;
            Eigen::Vector3d v;
            gtsam::Rot3     rot;
            Eigen::Vector3d body_rate;

            double          acc_z;
            Eigen::Vector3d torque;
        
            Eigen::Vector4d thrust_torque;
            Eigen::Array4d  motor_rpm;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };

        // Define a structure to hold 3D points
        struct Point3D 
        {
            float x, y, z;
        };
            
        // User Interface
        void displaySetup();
        
        void renderHistoryTrj();
        
        void renderHistoryOpt(std::vector<State> & trj, boost::optional<gtsam::Vector3&> err = boost::none,  
                                                        boost::optional<Features&> features = boost::none, 
                                                        boost::optional<gtsam::Vector3&> vicon_measurement = boost::none, 
                                                        boost::optional<gtsam::Vector3 &> rot_err = boost::none, 
                                                        boost::optional<std::vector<State> &> state_trj = boost::none);

        void drawQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot);

        void drawCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot);

        void drawLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end);

        void drawFrame(gtsam::Vector3 p, gtsam::Rot3 rot);
        
        void drawLidarCloud(Features & features);
        
        void renderPanel();

       // Dynamics related function

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

        // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
        void setInput(double u1, double u2, double u3, double u4);
        void setInput(gtsam::Vector4 thrust_torque);
        // Runs the actual dynamics simulation with a time step of dt
        void step(double dt);

        // ODE intergration based state propagation
        void operator()(const Quadrotor::stateType &x , Quadrotor::stateType &dxdt , const double t);
        void stepODE(double dt, gtsam::Vector4 fm);

        // Compute every rotor's rotation vel RPM
        Eigen::Vector4d CumputeRotorsVel();
        Eigen::Vector4d InvCumputeRotorsVel(Eigen::Vector4d rotor_speed);

        Eigen::Vector3d getAcc() const;

        gtsam::Vector3 getObs1();
        
        std::vector<gtsam::Vector3> getObsN();

    private:
        void printCurState();

        // Compute Control Allocation's effectiveness matrix
        Eigen::Matrix4d ComputeEffectivenessMatrix();

        std::vector<Point3D> generateSpherePoints(float radius, int numTheta, int numPhi);
        std::vector<Point3D> generatePointsOutsideCylinder(int numTheta, float radius, float height);
        Point3D              getEllipsePoint(uint8_t index);
        
        // Control Allocation's effectiveness matrix
        Eigen::Matrix4d effectiveness_;

        pangolin::GlFont *text_font = new pangolin::GlFont("../data/timr45w.ttf", 30.0);

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
        double esc_factor_;
        
        Eigen::Vector3d drag_force_params_;

        Quadrotor::State state_, last_state_;
        std::vector<Quadrotor::State> trj_;

        Eigen::Vector3d acc_;

        Eigen::Array4d  input_;
        Eigen::Vector3d external_force_;
        Eigen::Vector3d external_moment_;
        gtsam::Vector4  thrust_torque_;

        pangolin::View d_cam;
        std::shared_ptr<pangolin::OpenGlRenderState> s_cam;
        stringUI str_force_;
        stringUI str_M1_;
        stringUI str_M2_;
        stringUI str_M3_;
        stringUI str_Quad_x_;
        stringUI str_Quad_y_;
        stringUI str_Quad_z_;
        stringUI str_Quad_velx_;
        stringUI str_Quad_vely_;
        stringUI str_Quad_velz_;
        stringUI str_Quad_Rx_;
        stringUI str_Quad_Ry_;
        stringUI str_Quad_Rz_;
        stringUI str_AVE_ERR_;
        stringUI str_timestamp_;
        stringUI str_rotor_[4];

        std::vector<gtsam::Vector3> errs_;
        const uint64_t ERRS_LENS = 1;

        float axis_dist_;
        float propeller_dist_;
        const uint64_t HISTORY_TRJ_LENS = 1000;
        float trj_len_max_ = 1.0;
        int8_t obs_num_ = 1;

        std::default_random_engine generator_;
        std::ofstream record_info_;

        // force noise
        double THRUST_NOISE_MEAN = 0.0;
        double THRUST_NOISE_COV  = 0.0;
        double ANGULAR_SPEED_MEAN = 0.0;
        double ANGULAR_SPEED_COV = 0.0;

        Geometry geometry_;

        std::vector<Point3D> spherePoints_;
        Point3D              sphereCenter_;
        std::vector<Point3D> cylinderPoints_;
        std::vector<Point3D> obstacle_centers_;

        float clock_ = 0;
    };
}
#endif
