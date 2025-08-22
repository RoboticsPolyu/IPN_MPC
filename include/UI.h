#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <fstream>
#include <random>
#include <iomanip>
#include <sstream>

#include "gtsam_wrapper.h"
#include "env_sensors_sim/Common.h"

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>

#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/handler/handler.h>

namespace QuadrotorSim_SO3
{
    class UI
    {
    public:
        using StringUI = std::shared_ptr<pangolin::Var<std::string>>;
        using StateType = boost::array<double, 22>;
        
        UI(float trj_len_max, uint8_t obs_num, double che_dis);
        ~UI();

        // User Interface
        void displaySetup();
        void renderHistoryTrj();
        
        void renderHistoryOpt(State& state, 
                            std::vector<State>& pred_trj, 
                            boost::optional<gtsam::Vector3&> err = boost::none,  
                            boost::optional<Features&> features = boost::none, 
                            boost::optional<gtsam::Vector3&> vicon_measurement = boost::none, 
                            boost::optional<gtsam::Vector3&> rot_err = boost::none, 
                            boost::optional<std::vector<State>&> ref_trj = boost::none,
                            boost::optional<float&> opt_cost = boost::none,
                            boost::optional<std::vector<Obstacle>&> obstacle_centers = boost::none);
        
        gtsam::Vector3 getObs1() const;
        std::vector<gtsam::Vector3> getObstacles() const;

    private:
        // Drawing methods
        void drawQuadrotor(const gtsam::Vector3& p, const gtsam::Rot3& rot);
        void drawCircle(const gtsam::Vector3& color, float r, const gtsam::Vector3& center, const gtsam::Rot3& rot);
        void drawLine(const gtsam::Vector3& color, const gtsam::Vector3& begin, const gtsam::Vector3& end);
        void drawFrame(const gtsam::Vector3& p, const gtsam::Rot3& rot);
        void drawLidarCloud(Features& features);
        void drawTrjPoint(const gtsam::Vector3& p, float size = 3.0f, const gtsam::Vector3& color = gtsam::Vector3(0.1, 0.2, 0.7));
        void drawCollisionPoint(const gtsam::Vector3& p);
        
        // Utility methods
        void renderPanel();
        bool checkCollision(const State& state, const gtsam::Vector3& obstacle_center, float r) const;
        std::vector<Point3D> generateSpherePoints(float radius, int numTheta, int numPhi) const;
        void drawCylinder(const gtsam::Vector3& position, float radius, float height, 
                        const gtsam::Vector3& color, int segments = 32);
        void drawSphere(const gtsam::Vector3& position, float radius, const gtsam::Vector3& color, int numTheta, int numPhi);

        // UI components
        pangolin::View d_cam_;
        std::shared_ptr<pangolin::OpenGlRenderState> s_cam_;
        
        // UI strings
        StringUI str_force_;
        StringUI str_M1_, str_M2_, str_M3_;
        StringUI str_Quad_x_, str_Quad_y_, str_Quad_z_;
        StringUI str_Quad_velx_, str_Quad_vely_, str_Quad_velz_;
        StringUI str_AVE_ERR_;
        StringUI str_timestamp_;
        StringUI str_opt_cost_;
        StringUI str_rotor_[4];

        // Geometry data
        std::vector<Point3D> spherePoints_;
        std::vector<Point3D> obstacle_centers_;

        // State and trajectory
        State state_;
        std::vector<State> trj_;
        std::vector<gtsam::Vector3> errs_;

        // Parameters
        float axis_dist_;
        float propeller_dist_;
        float prop_radius_;
        float trj_len_max_;
        float opt_cost_;
        double che_dis_;
        float clock_;
        float delta_t_;
        
        uint8_t obs_num_;
        const uint64_t HISTORY_TRJ_LENS = 1000;
        const uint64_t ERRS_LENS = 1;

        // Resources
        std::ofstream record_info_;
        std::default_random_engine generator_;
        pangolin::GlFont* text_font_;
        
        // Constants
        static constexpr float SQRT2_INV = 1.0f / 1.41421356237f;

    };
}