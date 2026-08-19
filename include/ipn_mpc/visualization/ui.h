#pragma once

#include <array>
#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <ipn_mpc/factors/gtsam_compatibility.h>
#include <ipn_mpc/simulation/types.h>
#include <memory>
#include <pangolin/display/view.h>
#include <pangolin/var/var.h>
#include <string>
#include <vector>

namespace QuadrotorSim_SO3 {
class UI {
  public:
    using StringUI = std::shared_ptr<pangolin::Var<std::string>>;
    using ButtonUI = std::shared_ptr<pangolin::Var<bool>>;
    using StateType = boost::array<double, 22>;

    UI(float max_trajectory_length, uint8_t obstacle_count, double collision_distance,
       const std::string& trajectory_record_path = "../log/record_info.txt");
    ~UI() = default;

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
    struct Panel {
        StringUI force;
        std::array<StringUI, 3> torque;
        std::array<StringUI, 3> position;
        std::array<StringUI, 3> velocity;
        std::array<StringUI, 4> rotor;
        StringUI average_error;
        StringUI timestamp;
        StringUI optimization_cost;
        StringUI collision;
        StringUI clearance;
        StringUI obstacles;
        StringUI speed;
        StringUI solve_time;
        StringUI mean_solve_time;
        StringUI p95_solve_time;
        StringUI cpu;
        StringUI deadline_misses;
        StringUI simulation_state;
    };

    void beginFrame();
    void finishFrame();
    void appendHistory(const State& state);
    void recordState(const State& state, const gtsam::Vector3& position_error,
                     const boost::optional<gtsam::Vector3&>& rotation_error);
    void updateObstacleStatus(const State& state, const std::vector<Obstacle>& obstacles);
    void drawObstacles(const State& state, const std::vector<State>& prediction,
                       const std::vector<Obstacle>& obstacles);
    void drawReferenceTrajectory(const std::vector<State>& trajectory);
    void drawPredictedTrajectory(const std::vector<State>& trajectory);
    void drawHistoryTrajectory();
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

    void renderPanel();
    void updatePauseState();
    bool checkCollision(const State& state, const Obstacle& obstacle) const;
    void drawCylinder(const gtsam::Vector3& position, float radius, float height,
                      const gtsam::Vector3& color, int segments = 32);
    void drawSphere(const gtsam::Vector3& position, float radius, const gtsam::Vector3& color,
                    int numTheta, int numPhi);

    pangolin::View* camera_view_{nullptr};
    std::shared_ptr<pangolin::OpenGlRenderState> s_cam_;
    Panel panel_;
    ButtonUI stop_button_;
    ButtonUI continue_button_;
    std::vector<Point3D> obstacle_centers_;
    State state_;
    std::vector<State> trj_;
    std::vector<gtsam::Vector3> errs_;
    float trj_len_max_{0.0F};
    float opt_cost_{-1.0F};
    double collision_distance_{0.0};
    double minimum_clearance_{0.0};
    bool collision_active_{false};
    bool paused_{false};
    std::size_t visible_obstacles_{0};
    double solve_time_ms_{0.0}, mean_solve_time_ms_{0.0}, p95_solve_time_ms_{0.0},
        cpu_percent_{0.0};
    std::size_t deadline_misses_{0};
    float clock_{0.0F};
    static constexpr uint64_t kHistoryTrajectoryLength = 1000;
    static constexpr uint64_t kErrorHistoryLength = 1;
    std::ofstream record_info_;
};
} // namespace QuadrotorSim_SO3
