#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/visualization/ui.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <pangolin/display/default_font.h>
#include <pangolin/display/display.h>
#include <pangolin/display/widgets.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>
#include <pangolin/var/varextra.h>
#include <sstream>
#include <unistd.h>

namespace QuadrotorSim_SO3 {
namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kArmLength = 0.095F;
constexpr float kPropellerRadius = 0.042F;
constexpr float kSimulationStep = 0.01F;

gtsam::Vector3 obstacleColor(bool collision) {
    return collision ? gtsam::Vector3(1.0, 0.08, 0.08) : gtsam::Vector3(0.22, 0.68, 0.42);
}

void setText(const UI::StringUI& variable, double value, int precision = 6) {
    std::ostringstream stream;
    stream << std::setprecision(precision) << value;
    *variable = stream.str();
}

UI::StringUI makeText(const std::string& name, const std::string& initial) {
    return std::make_shared<pangolin::Var<std::string>>("ui." + name, initial);
}
} // namespace

UI::UI(float max_trajectory_length, uint8_t obstacle_count, double collision_distance,
       const std::string& trajectory_record_path)
    : trj_len_max_(max_trajectory_length), collision_distance_(collision_distance) {
    static_cast<void>(obstacle_count);
    const std::filesystem::path record_path(trajectory_record_path);
    std::error_code directory_error;
    if (!record_path.parent_path().empty())
        std::filesystem::create_directories(record_path.parent_path(), directory_error);
    record_info_.open(record_path);
    if (!record_info_.is_open()) {
        IPN_LOG_WARNING << "Could not open trajectory record file: " << record_path.string()
                        << (directory_error ? " (" + directory_error.message() + ")" : "");
    }

    displaySetup();
}

void UI::drawSphere(const gtsam::Vector3& position, float radius, const gtsam::Vector3& color,
                    int numTheta, int numPhi) {
    glColor3f(color[0], color[1], color[2]); // Set color (assumes normalized [0, 1])

    // Render the sphere using quads
    glBegin(GL_QUADS);
    for (int i = 0; i < numTheta; ++i) {
        float theta1 = 2.0f * M_PI * i / numTheta;
        float theta2 = 2.0f * M_PI * (i + 1) / numTheta;
        for (int j = 0; j < numPhi; ++j) {
            float phi1 = M_PI * j / numPhi;
            float phi2 = M_PI * (j + 1) / numPhi;

            // Compute vertex coordinates and normals
            gtsam::Vector3 v1(radius * std::sin(phi1) * std::cos(theta1),
                              radius * std::sin(phi1) * std::sin(theta1), radius * std::cos(phi1));
            gtsam::Vector3 v2(radius * std::sin(phi1) * std::cos(theta2),
                              radius * std::sin(phi1) * std::sin(theta2), radius * std::cos(phi1));
            gtsam::Vector3 v3(radius * std::sin(phi2) * std::cos(theta2),
                              radius * std::sin(phi2) * std::sin(theta2), radius * std::cos(phi2));
            gtsam::Vector3 v4(radius * std::sin(phi2) * std::cos(theta1),
                              radius * std::sin(phi2) * std::sin(theta1), radius * std::cos(phi2));

            // Normals (normalized vertex positions, as the sphere is centered at origin before
            // translation)
            gtsam::Vector3 n1 = v1 / radius;
            gtsam::Vector3 n2 = v2 / radius;
            gtsam::Vector3 n3 = v3 / radius;
            gtsam::Vector3 n4 = v4 / radius;

            // Define quad (counter-clockwise for correct facing)
            glNormal3f(n1[0], n1[1], n1[2]);
            glVertex3f(position[0] + v1[0], position[1] + v1[1], position[2] + v1[2]);
            glNormal3f(n2[0], n2[1], n2[2]);
            glVertex3f(position[0] + v2[0], position[1] + v2[1], position[2] + v2[2]);
            glNormal3f(n3[0], n3[1], n3[2]);
            glVertex3f(position[0] + v3[0], position[1] + v3[1], position[2] + v3[2]);
            glNormal3f(n4[0], n4[1], n4[2]);
            glVertex3f(position[0] + v4[0], position[1] + v4[1], position[2] + v4[2]);
        }
    }
    glEnd();
}

void UI::drawCylinder(const gtsam::Vector3& position, float radius, float height,
                      const gtsam::Vector3& color, int segments) {
    // Define gradient colors: darker at bottom, lighter at top
    gtsam::Vector3 dark_color = color * 0.7f;  // Darker shade (70% brightness)
    gtsam::Vector3 light_color = color * 1.3f; // Lighter shade (130% brightness)
    light_color = light_color.cwiseMin(gtsam::Vector3(1.0f, 1.0f, 1.0f)); // Cap at 1.0

    // Draw bottom circle (filled, at position.z, using dark_color)
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, -1.0f); // Normal pointing downward
    glColor3f(dark_color[0], dark_color[1], dark_color[2]);
    for (int i = 0; i < segments; ++i) {
        float theta = 2.0f * M_PI * i / segments;
        float x = radius * std::cos(theta);
        float y = radius * std::sin(theta);
        glVertex3f(position[0] + x, position[1] + y, position[2]);
    }
    glEnd();

    // Draw top circle (filled, at position.z + height, using light_color)
    glBegin(GL_POLYGON);
    glNormal3f(0.0f, 0.0f, 1.0f); // Normal pointing upward
    glColor3f(light_color[0], light_color[1], light_color[2]);
    for (int i = 0; i < segments; ++i) {
        float theta = 2.0f * M_PI * i / segments;
        float x = radius * std::cos(theta);
        float y = radius * std::sin(theta);
        glVertex3f(position[0] + x, position[1] + y, position[2] + height);
    }
    glEnd();

    // Draw side surface (filled quads with gradient)
    glBegin(GL_QUADS);
    for (int i = 0; i < segments; ++i) {
        float theta1 = 2.0f * M_PI * i / segments;
        float theta2 = 2.0f * M_PI * (i + 1) / segments;

        float x1 = radius * std::cos(theta1);
        float y1 = radius * std::sin(theta1);
        float x2 = radius * std::cos(theta2);
        float y2 = radius * std::sin(theta2);

        // Compute normal for the side surface (outward from cylinder axis)
        gtsam::Vector3 normal1(std::cos(theta1), std::sin(theta1), 0.0f);
        glNormal3f(normal1[0], normal1[1], normal1[2]);

        // Define quad vertices with gradient colors
        // Bottom vertices (use dark_color)
        glColor3f(dark_color[0], dark_color[1], dark_color[2]);
        glVertex3f(position[0] + x1, position[1] + y1, position[2]); // Bottom left
        glVertex3f(position[0] + x2, position[1] + y2, position[2]); // Bottom right

        // Top vertices (use light_color)
        glColor3f(light_color[0], light_color[1], light_color[2]);
        glVertex3f(position[0] + x2, position[1] + y2, position[2] + height); // Top right
        glVertex3f(position[0] + x1, position[1] + y1, position[2] + height); // Top left
    }
    glEnd();
}

void UI::displaySetup() {
    pangolin::CreateWindowAndBind("IPN MPC | Quadrotor Simulation", 1600, 900);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glClearColor(0.025F, 0.040F, 0.075F, 1.0F);

    // Camera setup with adjusted view to ensure visibility
    s_cam_ = std::make_shared<pangolin::OpenGlRenderState>(
        pangolin::ProjectionMatrix(1600, 900, 850, 850, 800, 450, 0.05, 1000),
        pangolin::ModelViewLookAt(3.8, -4.5, 3.2, 0, 0, 0.8, 0.0, 0.0, 1.0));

    const int UI_WIDTH = 24 * pangolin::default_font().MaxWidth();
    camera_view_ = &pangolin::CreateDisplay();
    camera_view_->SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 1600.0F / 900.0F)
        .SetHandler(new pangolin::Handler3D(*s_cam_));

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    stop_button_ = std::make_shared<pangolin::Var<bool>>("ui.Stop", false, false);
    continue_button_ = std::make_shared<pangolin::Var<bool>>("ui.Continue", false, false);
    panel_.simulation_state = makeText("SIMULATION", "RUNNING");
    panel_.force = makeText("Force(N)", "Force");
    const std::array<std::string, 3> axes = {"x", "y", "z"};
    for (std::size_t i = 0; i < axes.size(); ++i) {
        panel_.torque[i] = makeText("M" + std::to_string(i + 1) + "(N*m)", "M" + std::to_string(i + 1));
        panel_.position[i] = makeText("UAV" + axes[i] + "(m)", "UAV" + axes[i]);
        panel_.velocity[i] = makeText("UAV_v" + axes[i] + "(m/s)", "UAV_v" + axes[i]);
    }
    for (std::size_t i = 0; i < panel_.rotor.size(); ++i) {
        panel_.rotor[i] = makeText("ROTOR" + std::to_string(i + 1) + "(RPM)",
                                   "ROTOR" + std::to_string(i + 1));
    }
    panel_.average_error = makeText("AVE_ERR(m)", "AVE_ERR");
    panel_.timestamp = makeText("TIMESTAMP(s)", "TIMESTAMP");
    panel_.optimization_cost = makeText("OPT_COST(s)", "OPT_COST");
    panel_.collision = makeText("COLLISION", "CLEAR");
    panel_.clearance = makeText("MIN_CLEARANCE(m)", "n/a");
    panel_.obstacles = makeText("OBSTACLES", "0");
    panel_.speed = makeText("SPEED(m/s)", "0");
    panel_.solve_time = makeText("SOLVE_TIME(ms)", "0");
    panel_.mean_solve_time = makeText("SOLVE_MEAN(ms)", "0");
    panel_.p95_solve_time = makeText("SOLVE_P95(ms)", "0");
    panel_.cpu = makeText("CPU(%)", "0");
    panel_.deadline_misses = makeText("DEADLINE_MISSES", "0");
}

void UI::setPerformanceStats(double solve_time_ms, double mean_solve_time_ms,
                             double p95_solve_time_ms, double cpu_percent,
                             std::size_t deadline_misses) {
    solve_time_ms_ = solve_time_ms;
    mean_solve_time_ms_ = mean_solve_time_ms;
    p95_solve_time_ms_ = p95_solve_time_ms;
    cpu_percent_ = cpu_percent;
    deadline_misses_ = deadline_misses;
}

void UI::drawTrjPoint(const gtsam::Vector3& p, float size, const gtsam::Vector3& color) {
    glPointSize(size);
    glBegin(GL_POINTS);
    glColor3f(color[0], color[1], color[2]);
    glVertex3f(p[0], p[1], p[2]);
    glEnd();
}

void UI::drawCollisionPoint(const gtsam::Vector3& p) {
    drawTrjPoint(p, 12.0f, gtsam::Vector3(1.0, 0.2, 0.1));
}

void UI::drawGroundPlane(float half_extent, float grid_spacing) {
    glDepthMask(GL_FALSE);
    glColor4f(0.055F, 0.085F, 0.13F, 0.72F);
    glBegin(GL_QUADS);
    glVertex3f(-half_extent, -half_extent, 0.0F);
    glVertex3f(half_extent, -half_extent, 0.0F);
    glVertex3f(half_extent, half_extent, 0.0F);
    glVertex3f(-half_extent, half_extent, 0.0F);
    glEnd();
    glDepthMask(GL_TRUE);

    glLineWidth(1.0F);
    for (float coordinate = -half_extent; coordinate <= half_extent + 1.0e-4F;
         coordinate += grid_spacing) {
        const bool major = std::abs(std::remainder(coordinate, 1.0F)) < 1.0e-4F;
        const float brightness = major ? 0.25F : 0.14F;
        glColor4f(brightness, brightness + 0.04F, brightness + 0.09F,
                  major ? 0.75F : 0.5F);
        glBegin(GL_LINES);
        glVertex3f(coordinate, -half_extent, 0.002F);
        glVertex3f(coordinate, half_extent, 0.002F);
        glVertex3f(-half_extent, coordinate, 0.002F);
        glVertex3f(half_extent, coordinate, 0.002F);
        glEnd();
    }
}

void UI::drawVehicleShadow(const gtsam::Vector3& p) {
    const float altitude = std::max(0.0, p.z());
    const float radius = 0.09F + 0.025F * std::min(altitude, 3.0F);
    const float alpha = 0.38F / (1.0F + 0.45F * altitude);
    glColor4f(0.0F, 0.0F, 0.0F, alpha);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(p.x(), p.y(), 0.006F);
    for (int i = 0; i <= 48; ++i) {
        const float angle = 2.0F * static_cast<float>(M_PI) * i / 48.0F;
        glVertex3f(p.x() + radius * std::cos(angle), p.y() + radius * std::sin(angle), 0.006F);
    }
    glEnd();
}

void UI::drawQuadrotor(const gtsam::Vector3& p, const gtsam::Rot3& rot) {
    const float motor_distance = kArmLength;
    const std::vector<gtsam::Vector3> motor_offsets = {
        {motor_distance, motor_distance, 0.0}, {-motor_distance, -motor_distance, 0.0},
        {-motor_distance, motor_distance, 0.0}, {motor_distance, -motor_distance, 0.0}};

    // Carbon arms with bright motor pods make attitude readable at a distance.
    glLineWidth(6.0F);
    for (std::size_t i = 0; i < motor_offsets.size(); ++i) {
        const gtsam::Vector3 center = rot.rotate(motor_offsets[i]) + p;
        drawLine(i < 2 ? gtsam::Vector3(0.08, 0.72, 0.95)
                       : gtsam::Vector3(0.18, 0.24, 0.32),
                 p, center);
    }

    // Solid center fuselage, transformed by the vehicle pose.
    const gtsam::Matrix3 rotation = rot.matrix();
    GLdouble transform[16] = {
        rotation(0, 0), rotation(1, 0), rotation(2, 0), 0.0,
        rotation(0, 1), rotation(1, 1), rotation(2, 1), 0.0,
        rotation(0, 2), rotation(1, 2), rotation(2, 2), 0.0,
        p.x(),          p.y(),          p.z(),          1.0};
    glPushMatrix();
    glMultMatrixd(transform);
    glScaled(0.10, 0.075, 0.035);
    pangolin::glDrawColouredCube();
    glPopMatrix();

    for (std::size_t i = 0; i < motor_offsets.size(); ++i) {
        const gtsam::Vector3 center = rot.rotate(motor_offsets[i]) + p;
        drawSphere(center, 0.018F, gtsam::Vector3(0.15, 0.18, 0.23), 12, 8);

        // Translucent filled rotor discs suggest fast-spinning propellers.
        const gtsam::Vector3 rotor_color =
            i < 2 ? gtsam::Vector3(0.1, 0.8, 1.0) : gtsam::Vector3(1.0, 0.35, 0.18);
        glColor4f(rotor_color.x(), rotor_color.y(), rotor_color.z(), 0.24F);
        glBegin(GL_TRIANGLE_FAN);
        glVertex3d(center.x(), center.y(), center.z());
        for (int segment = 0; segment <= 32; ++segment) {
            const double angle = 2.0 * M_PI * segment / 32.0;
            const gtsam::Vector3 local(kPropellerRadius * std::cos(angle),
                                       kPropellerRadius * std::sin(angle), 0.0);
            const gtsam::Vector3 vertex = center + rot.rotate(local);
            glVertex3d(vertex.x(), vertex.y(), vertex.z());
        }
        glEnd();
        glLineWidth(1.5F);
        drawCircle(rotor_color, kPropellerRadius, center, rot);
    }

    drawFrame(p, rot);
}

void UI::drawCircle(const gtsam::Vector3& color, float r, const gtsam::Vector3& center,
                    const gtsam::Rot3& rot) {
    glColor3f(color[0], color[1], color[2]); // Already normalized

    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 360; ++i) {
        float angle = static_cast<float>(i) * M_PI / 180.0f;
        gtsam::Vector3 local_point(r * cos(angle), r * sin(angle), 0);
        gtsam::Vector3 world_point = rot.rotate(local_point) + center;
        glVertex3f(world_point[0], world_point[1], world_point[2]);
    }
    glEnd();
}

void UI::drawLine(const gtsam::Vector3& color, const gtsam::Vector3& begin,
                  const gtsam::Vector3& end) {
    glBegin(GL_LINES);
    glColor3f(color[0], color[1], color[2]);
    glVertex3d(begin[0], begin[1], begin[2]);
    glVertex3d(end[0], end[1], end[2]);
    glEnd();
}

void UI::drawLidarCloud(Features& features) {
    for (const auto type : {PointType::L_NONV, PointType::L_VIS}) {
        glPointSize(type == PointType::L_VIS ? 5.0F : 3.0F);
        const gtsam::Vector3 color = type == PointType::L_VIS
                                         ? gtsam::Vector3(0.1, 0.2, 0.7)
                                         : gtsam::Vector3(0.1, 0.8, 0.7);
        glColor3d(color.x(), color.y(), color.z());
        glBegin(GL_POINTS);
        for (const auto& feature : features) {
            if (feature.type == type) glVertex3d(feature.x, feature.y, feature.z);
        }
        glEnd();
    }
}

void UI::drawFrame(const gtsam::Vector3& p, const gtsam::Rot3& rot) {
    const float axis_length = 0.1f;
    const std::vector<std::pair<gtsam::Vector3, gtsam::Vector3>> axes = {
        {gtsam::Vector3(1.0f, 0, 0), gtsam::Vector3(axis_length, 0, 0)}, // Normalized colors
        {gtsam::Vector3(0, 1.0f, 0), gtsam::Vector3(0, axis_length, 0)},
        {gtsam::Vector3(0, 0, 1.0f), gtsam::Vector3(0, 0, axis_length)}};

    for (const auto& axis : axes) {
        gtsam::Vector3 end = rot.rotate(axis.second) + p;
        drawLine(axis.first, p, end);
    }
}

bool UI::renderHistoryTrj(const State& state) {
    if (pangolin::ShouldQuit()) {
        return false;
    }

    state_ = state;
    appendHistory(state_);
    do {
        updatePauseState();
        beginFrame();
        glLineWidth(2);
        drawGroundPlane();
        drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::Identity());
        for (std::size_t i = 1; i < trj_.size(); ++i)
            drawLine(gtsam::Vector3(0.15, 0.82, 0.95), trj_[i - 1].p, trj_[i].p);
        drawVehicleShadow(state_.p);
        drawQuadrotor(state_.p, state_.rot);
        finishFrame();
        updatePauseState();
        if (paused_) usleep(16000);
    } while (paused_ && !pangolin::ShouldQuit());
    usleep(100);
    return !pangolin::ShouldQuit();
}

void UI::beginFrame() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera_view_->Activate(*s_cam_);
}

void UI::finishFrame() {
    renderPanel();
    pangolin::FinishFrame();
}

void UI::appendHistory(const State& state) {
    if (trj_.size() >= kHistoryTrajectoryLength) trj_.erase(trj_.begin());
    trj_.push_back(state);
}

gtsam::Vector3 UI::getObs1() const {
    return obstacle_centers_.empty()
               ? gtsam::Vector3::Zero()
               : gtsam::Vector3(obstacle_centers_[0].x, obstacle_centers_[0].y,
                                obstacle_centers_[0].z);
}

std::vector<gtsam::Vector3> UI::getObstacles() const {
    std::vector<gtsam::Vector3> obstacles;
    obstacles.reserve(obstacle_centers_.size());
    for (const auto& center : obstacle_centers_) {
        obstacles.emplace_back(center.x, center.y, center.z);
    }
    return obstacles;
}

bool UI::checkCollision(const State& state, const Obstacle& obstacle) const {
    const double uav_radius = collision_distance_;
    if (obstacle.obs_type == ObsType::sphere) {
        return (state.p - obstacle.obs_pos).norm() <= uav_radius + obstacle.obs_size;
        // IPN_LOG_DEBUG << "Collision detected: " << obs_distance;
        // IPN_LOG_DEBUG << "State p: " << state.p.transpose();
        // IPN_LOG_DEBUG << "Obstacle center: " << obstacle_center.transpose();
    } else if (obstacle.obs_type == ObsType::cylinder) {
        const double radial = (state.p.head<2>() - obstacle.obs_pos.head<2>()).norm();
        const bool vertical = state.p.z() + uav_radius >= obstacle.obs_pos.z() &&
                              state.p.z() - uav_radius <= obstacle.obs_pos.z() + obstacle.obs_height;
        return radial <= uav_radius + obstacle.obs_size && vertical;
    } else if (obstacle.obs_type == ObsType::box) {
        const gtsam::Vector3 closest = state.p.cwiseMax(obstacle.obs_pos - obstacle.half_extents)
                                             .cwiseMin(obstacle.obs_pos + obstacle.half_extents);
        return (state.p - closest).norm() <= uav_radius;
    } else {
        IPN_LOG_DEBUG << "Collision check skipped: unsupported_obstacle_type="
                      << static_cast<int>(obstacle.obs_type);
    }
    return false;
}

void UI::recordState(const State& state, const gtsam::Vector3& position_error,
                     const boost::optional<gtsam::Vector3&>& rotation_error) {
    if (!record_info_) return;
    record_info_ << state.p.transpose() << ' ' << position_error.transpose() << ' '
                 << state.thrust_torque.transpose();
    if (rotation_error) record_info_ << ' ' << rotation_error->transpose();
    record_info_ << '\n';
}

void UI::updateObstacleStatus(const State& state, const std::vector<Obstacle>& obstacles) {
    collision_active_ = false;
    minimum_clearance_ = std::numeric_limits<double>::infinity();
    visible_obstacles_ = obstacles.size();
    for (const auto& obstacle : obstacles) {
        collision_active_ = collision_active_ || checkCollision(state, obstacle);
        minimum_clearance_ = std::min(minimum_clearance_,
            (state.p - obstacle.obs_pos).norm() - collision_distance_ - obstacle.obs_size);
    }
}

void UI::drawObstacles(const State& state, const std::vector<State>& prediction,
                       const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        const bool collision = checkCollision(state, obstacle);
        const gtsam::Vector3 color = obstacleColor(collision);
        switch (obstacle.obs_type) {
        case ObsType::sphere:
            drawSphere(obstacle.obs_pos, obstacle.obs_size, color, 32, 24);
            break;
        case ObsType::cylinder:
            drawCylinder(obstacle.obs_pos, obstacle.obs_size, obstacle.obs_height,
                         collision ? color : gtsam::Vector3(0.20, 0.38, 0.82), 32);
            break;
        case ObsType::box:
            glColor3d(color.x(), color.y(), color.z());
            glPushMatrix();
            glTranslated(obstacle.obs_pos.x(), obstacle.obs_pos.y(), obstacle.obs_pos.z());
            glScaled(2.0 * obstacle.half_extents.x(), 2.0 * obstacle.half_extents.y(),
                     2.0 * obstacle.half_extents.z());
            pangolin::glDrawColouredCube();
            glPopMatrix();
            break;
        default:
            break;
        }
        for (const auto& predicted_state : prediction) {
            if (checkCollision(predicted_state, obstacle)) drawCollisionPoint(predicted_state.p);
        }
    }
}

void UI::drawReferenceTrajectory(const std::vector<State>& trajectory) {
    glLineWidth(1.5F);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        drawTrjPoint(trajectory[i].p, 4.0F, gtsam::Vector3(0.2, 0.85, 1.0));
        if (i > 0) drawLine(gtsam::Vector3(0.12, 0.55, 0.72), trajectory[i - 1].p,
                            trajectory[i].p);
    }
}

void UI::drawPredictedTrajectory(const std::vector<State>& trajectory) {
    glLineWidth(3.5F);
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
        const double progress = static_cast<double>(i) / trajectory.size();
        drawLine(gtsam::Vector3(1.0, 0.72 - 0.35 * progress, 0.12), trajectory[i - 1].p,
                 trajectory[i].p);
    }
}

void UI::drawHistoryTrajectory() {
    float length = 0.0F;
    glLineWidth(3.0F);
    for (std::size_t i = trj_.size(); i > 1; --i) {
        const auto& current = trj_[i - 1].p;
        const auto& previous = trj_[i - 2].p;
        length += static_cast<float>((current - previous).norm());
        if (length >= trj_len_max_) break;
        const float fade = std::max(0.25F, 1.0F - length / trj_len_max_);
        drawLine(gtsam::Vector3(0.35 * fade, 0.95 * fade, 0.82 * fade), current, previous);
    }
}

bool UI::renderHistoryOpt(State& state, std::vector<State>& pred_trj,
                          boost::optional<gtsam::Vector3&> err, boost::optional<Features&> features,
                          boost::optional<gtsam::Vector3&> vicon_measurement,
                          boost::optional<gtsam::Vector3&> rot_err,
                          boost::optional<std::vector<State>&> ref_trj,
                          boost::optional<float&> opt_cost,
                          boost::optional<std::vector<Obstacle>&> obstacle_centers) {
    clock_ += kSimulationStep;
    state_ = state;
    if (opt_cost) opt_cost_ = *opt_cost;
    const gtsam::Vector3 position_error = err ? *err : gtsam::Vector3::Zero();
    errs_.push_back(position_error);
    if (errs_.size() > kErrorHistoryLength) errs_.erase(errs_.begin());
    recordState(state_, position_error, rot_err);

    if (pangolin::ShouldQuit()) {
        return false;
    }

    appendHistory(state_);
    do {
        updatePauseState();
        beginFrame();
        drawGroundPlane();
        drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::Identity());
        if (ref_trj) drawReferenceTrajectory(*ref_trj);
        drawPredictedTrajectory(pred_trj);
        if (obstacle_centers) {
            updateObstacleStatus(state, *obstacle_centers);
            drawObstacles(state, pred_trj, *obstacle_centers);
        } else {
            updateObstacleStatus(state, {});
        }
        drawHistoryTrajectory();
        drawVehicleShadow(state_.p);
        drawQuadrotor(state_.p, state_.rot);
        if (features) drawLidarCloud(*features);
        if (vicon_measurement)
            drawTrjPoint(*vicon_measurement, 10.0F, gtsam::Vector3(0.6, 0.2, 0.5));
        finishFrame();
        updatePauseState();
        if (paused_) usleep(16000);
    } while (paused_ && !pangolin::ShouldQuit());
    usleep(1000);
    return !pangolin::ShouldQuit();
}

void UI::updatePauseState() {
    if (stop_button_ && pangolin::Pushed(*stop_button_)) paused_ = true;
    if (continue_button_ && pangolin::Pushed(*continue_button_)) paused_ = false;
    if (panel_.simulation_state) *panel_.simulation_state = paused_ ? "PAUSED" : "RUNNING";
}

void UI::renderPanel() {
    setText(panel_.force, state_.thrust_torque[0]);
    for (std::size_t i = 0; i < panel_.torque.size(); ++i) {
        setText(panel_.torque[i], state_.thrust_torque[i + 1]);
        setText(panel_.position[i], state_.p[i]);
        setText(panel_.velocity[i], state_.v[i]);
    }
    for (std::size_t i = 0; i < panel_.rotor.size(); ++i)
        setText(panel_.rotor[i], state_.motor_rpm[i]);

    setText(panel_.timestamp, clock_);
    setText(panel_.optimization_cost, opt_cost_);
    setText(panel_.speed, state_.v.norm(), 4);
    setText(panel_.solve_time, solve_time_ms_, 4);
    setText(panel_.mean_solve_time, mean_solve_time_ms_, 4);
    setText(panel_.p95_solve_time, p95_solve_time_ms_, 4);
    setText(panel_.cpu, cpu_percent_, 4);
    *panel_.deadline_misses = std::to_string(deadline_misses_);
    *panel_.collision = collision_active_ ? "COLLISION" : "CLEAR";
    *panel_.obstacles = std::to_string(visible_obstacles_);
    if (std::isfinite(minimum_clearance_)) setText(panel_.clearance, minimum_clearance_, 4);
    else *panel_.clearance = "n/a";

    double error_sum = 0.0;
    for (const auto& error : errs_) error_sum += error.squaredNorm();
    const double average_error = errs_.empty() ? 0.0 : std::sqrt(error_sum / errs_.size());
    setText(panel_.average_error, average_error);
}
} // namespace QuadrotorSim_SO3
