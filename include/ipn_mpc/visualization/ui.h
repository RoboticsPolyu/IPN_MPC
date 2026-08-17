#pragma once

#include <Eigen/Core>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <ipn_mpc/factors/gtsam_compatibility.h>
#include <ipn_mpc/simulation/types.h>
#include <memory>
#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/display/widgets.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <random>
#include <sstream>
#include <vector>

namespace QuadrotorSim_SO3 {
class UI {
  public:
    using StringUI = std::shared_ptr<pangolin::Var<std::string>>;
    using StateType = boost::array<double, 22>;

    UI(float max_trajectory_length, uint8_t obstacle_count, double collision_distance,
       const std::string& trajectory_record_path = "../log/record_info.txt");
    ~UI() = default;

    // User Interface
    void displaySetup();
    bool renderHistoryTrj(const State& state);

    bool renderHistoryOpt(State& state, std::vector<State>& pred_trj,
                          boost::optional<gtsam::Vector3&> err = boost::none,
                          boost::optional<Features&> features = boost::none,
                          boost::optional<gtsam::Vector3&> vicon_measurement = boost::none,
                          boost::optional<gtsam::Vector3&> rot_err = boost::none,
                          boost::optional<std::vector<State>&> ref_trj = boost::none,
                          boost::optional<float&> opt_cost = boost::none,
                          boost::optional<std::vector<Obstacle>&> obstacle_centers = boost::none);

    gtsam::Vector3 getObs1() const;
    std::vector<gtsam::Vector3> getObstacles() const;
    void setPerformanceStats(double solve_time_ms, double mean_solve_time_ms,
                             double p95_solve_time_ms, double cpu_percent,
                             std::size_t deadline_misses);

  private:
    // Drawing methods
    void drawQuadrotor(const gtsam::Vector3& p, const gtsam::Rot3& rot);
    void drawGroundPlane(float half_extent = 5.0F, float grid_spacing = 0.5F);
    void drawVehicleShadow(const gtsam::Vector3& p);
    void drawCircle(const gtsam::Vector3& color, float r, const gtsam::Vector3& center,
                    const gtsam::Rot3& rot);
    void drawLine(const gtsam::Vector3& color, const gtsam::Vector3& begin,
                  const gtsam::Vector3& end);
    void drawFrame(const gtsam::Vector3& p, const gtsam::Rot3& rot);
    void drawLidarCloud(Features& features);
    void drawTrjPoint(const gtsam::Vector3& p, float size = 3.0f,
                      const gtsam::Vector3& color = gtsam::Vector3(0.1, 0.2, 0.7));
    void drawCollisionPoint(const gtsam::Vector3& p);

    // Utility methods
    void renderPanel();
    bool checkCollision(const State& state, const Obstacle& obstacle) const;
    std::vector<Point3D> generateSpherePoints(float radius, int numTheta, int numPhi) const;
    void drawCylinder(const gtsam::Vector3& position, float radius, float height,
                      const gtsam::Vector3& color, int segments = 32);
    void drawSphere(const gtsam::Vector3& position, float radius, const gtsam::Vector3& color,
                    int numTheta, int numPhi);

    // UI components
    pangolin::View* camera_view_{nullptr};
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam_;

    // UI strings
    StringUI str_force_;
    StringUI str_M1_, str_M2_, str_M3_;
    StringUI str_Quad_x_, str_Quad_y_, str_Quad_z_;
    StringUI str_Quad_velx_, str_Quad_vely_, str_Quad_velz_;
    StringUI str_AVE_ERR_;
    StringUI str_timestamp_;
    StringUI str_opt_cost_;
    StringUI str_collision_, str_clearance_, str_obstacles_, str_speed_;
    StringUI str_solve_time_, str_mean_solve_time_, str_p95_solve_time_, str_cpu_,
        str_deadline_misses_;
    StringUI str_rotor_[4];

    // Geometry data
    std::vector<Point3D> sphere_points_;
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
    double collision_distance_;
    double minimum_clearance_{0.0};
    bool collision_active_{false};
    std::size_t visible_obstacles_{0};
    double solve_time_ms_{0.0}, mean_solve_time_ms_{0.0}, p95_solve_time_ms_{0.0},
        cpu_percent_{0.0};
    std::size_t deadline_misses_{0};
    float clock_;
    float delta_t_;

    uint8_t obs_num_;
    static constexpr uint64_t kHistoryTrajectoryLength = 1000;
    static constexpr uint64_t kErrorHistoryLength = 1;

    // Resources
    std::ofstream record_info_;
    std::default_random_engine generator_;

    // Constants
    static constexpr float kInverseSqrtTwo = 1.0f / 1.41421356237f;
};
} // namespace QuadrotorSim_SO3
