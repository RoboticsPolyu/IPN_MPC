#include <iostream>

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
#include <pangolin/display/default_font.h>
#include <pangolin/handler/handler.h>

#include <vector>

namespace QuadrotorSim_SO3
{
    class UI
    {
    public:
        using stringUI = std::shared_ptr<pangolin::Var<std::string> >;
        using stateType = boost::array<double, 22>;
        
        UI::UI(float trj_len_max, uint8_t obs_num);

        // User Interface
        void displaySetup();
        
        void renderHistoryTrj();
        
        void renderHistoryOpt(State& state, std::vector<State> & pred_trj, boost::optional<gtsam::Vector3&> err = boost::none,  
                                                        boost::optional<Features&> features = boost::none, 
                                                        boost::optional<gtsam::Vector3&> vicon_measurement = boost::none, 
                                                        boost::optional<gtsam::Vector3 &> rot_err = boost::none, 
                                                        boost::optional<std::vector<State> &> ref_trj = boost::none);
        
        gtsam::Vector3 getObs1();
        
        std::vector<gtsam::Vector3> getObsN();

    private:

        void drawQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot);

        void drawCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot);

        void drawLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end);

        void drawFrame(gtsam::Vector3 p, gtsam::Rot3 rot);
        
        void drawLidarCloud(Features & features);
        
        void renderPanel();

        std::vector<Point3D> generateSpherePoints(float radius, int numTheta, int numPhi);
        std::vector<Point3D> generatePointsOutsideCylinder(int numTheta, float radius, float height);
        Point3D              getEllipsePoint(uint8_t index);
        pangolin::GlFont *text_font = new pangolin::GlFont("../data/timr45w.ttf", 30.0);   


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

        std::vector<Point3D> spherePoints_;
        Point3D              sphereCenter_;
        std::vector<Point3D> cylinderPoints_;
        std::vector<Point3D> obstacle_centers_;

        float axis_dist_;
        float propeller_dist_;
        const uint64_t HISTORY_TRJ_LENS = 1000;
        float trj_len_max_ = 1.0;
        uint8_t obs_num_ = 1;

        std::default_random_engine generator_;
        
        float clock_ = 0;
        std::ofstream record_info_;
        std::vector<State> trj_;
        
        State state_;
        float delta_t = 0.01f;
        Eigen::Array4d  input_;

        std::vector<gtsam::Vector3> errs_;
        const uint64_t ERRS_LENS = 1;
        
        double prop_radius_;
    };
}

