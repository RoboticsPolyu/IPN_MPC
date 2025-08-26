#include "UI.h"

namespace QuadrotorSim_SO3
{

    // Constants
    constexpr float UI::SQRT2_INV;

    UI::UI(float trj_len_max, uint8_t obs_num, double che_dis)
        : trj_len_max_(trj_len_max), obs_num_(obs_num), che_dis_(che_dis),
          clock_(0.0f), delta_t_(0.01f), opt_cost_(-1.0f),
          axis_dist_(0.10f), propeller_dist_(0.04f), prop_radius_(0.042f)
    {
        // Generate geometry
        spherePoints_ = generateSpherePoints(che_dis_, 100, 100);

        // Initialize recording
        record_info_.open("../data/record_info.txt");
        if (!record_info_.is_open()) {
            std::cerr << "Warning: Could not open record file" << std::endl;
        }

        // Initialize font
        text_font_ = new pangolin::GlFont("../data/timr45w.ttf", 30.0);

        displaySetup();
    }

    UI::~UI()
    {
        record_info_.close();
        delete text_font_;
    }

    void UI::drawSphere(const gtsam::Vector3& position, float radius, const gtsam::Vector3& color, int numTheta, int numPhi)
    {
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
                gtsam::Vector3 v1(
                    radius * std::sin(phi1) * std::cos(theta1),
                    radius * std::sin(phi1) * std::sin(theta1),
                    radius * std::cos(phi1)
                );
                gtsam::Vector3 v2(
                    radius * std::sin(phi1) * std::cos(theta2),
                    radius * std::sin(phi1) * std::sin(theta2),
                    radius * std::cos(phi1)
                );
                gtsam::Vector3 v3(
                    radius * std::sin(phi2) * std::cos(theta2),
                    radius * std::sin(phi2) * std::sin(theta2),
                    radius * std::cos(phi2)
                );
                gtsam::Vector3 v4(
                    radius * std::sin(phi2) * std::cos(theta1),
                    radius * std::sin(phi2) * std::sin(theta1),
                    radius * std::cos(phi2)
                );

                // Normals (normalized vertex positions, as the sphere is centered at origin before translation)
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
                         const gtsam::Vector3& color, int segments)
    {
        // Define gradient colors: darker at bottom, lighter at top
        gtsam::Vector3 dark_color = color * 0.7f; // Darker shade (70% brightness)
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

    void UI::displaySetup()
    {
        pangolin::CreateWindowAndBind("Model Predictive Control based on FGO", 1600, 800);

        // Camera setup with adjusted view to ensure visibility
        s_cam_ = std::make_shared<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(1600, 800, 800, 800, 800, 400, 0.1, 1000),
            pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, 0.0, 0.0, 1.0)); // Adjusted to view origin from angle

        const int UI_WIDTH = 20 * pangolin::default_font().MaxWidth();
        d_cam_ = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 1600.0f / 800.0f)
            .SetHandler(new pangolin::Handler3D(*s_cam_));

        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

        // Initialize UI variables
        str_force_ = std::make_shared<pangolin::Var<std::string>>("ui.Force(N)", "Force");
        str_M1_ = std::make_shared<pangolin::Var<std::string>>("ui.M1(N*m)", "M1");
        str_M2_ = std::make_shared<pangolin::Var<std::string>>("ui.M2(N*m)", "M2");
        str_M3_ = std::make_shared<pangolin::Var<std::string>>("ui.M3(N*m)", "M3");
        
        str_Quad_x_ = std::make_shared<pangolin::Var<std::string>>("ui.UAVx(m)", "UAVx");
        str_Quad_y_ = std::make_shared<pangolin::Var<std::string>>("ui.UAVy(m)", "UAVy");
        str_Quad_z_ = std::make_shared<pangolin::Var<std::string>>("ui.UAVz(m)", "UAVz");
        
        str_Quad_velx_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vx(m/s)", "UAV_vx");
        str_Quad_vely_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vy(m/s)", "UAV_vy");
        str_Quad_velz_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vz(m/s)", "UAV_vz");
        
        str_AVE_ERR_ = std::make_shared<pangolin::Var<std::string>>("ui.AVE_ERR(m)", "AVE_ERR");
        str_timestamp_ = std::make_shared<pangolin::Var<std::string>>("ui.TIMESTAMP(s)", "TIMESTAMP");
        str_opt_cost_ = std::make_shared<pangolin::Var<std::string>>("ui.OPT_COST(s)", "OPT_COST");
        
        for (int i = 0; i < 4; ++i) {
            str_rotor_[i] = std::make_shared<pangolin::Var<std::string>>(
                "ui.ROTOR" + std::to_string(i+1) + "(RPM)", "ROTOR" + std::to_string(i+1));
        }
    }

    void UI::drawTrjPoint(const gtsam::Vector3& p, float size, const gtsam::Vector3& color)
    {
        glPointSize(size);
        glBegin(GL_POINTS);
        glColor3f(color[0], color[1], color[2]);
        glVertex3f(p[0], p[1], p[2]);
        glEnd();
    }

    void UI::drawCollisionPoint(const gtsam::Vector3& p)
    {
        drawTrjPoint(p, 10.0f, gtsam::Vector3(1.0, 1.0, 0.0));  // yellow
    }

    void UI::drawQuadrotor(const gtsam::Vector3& p, const gtsam::Rot3& rot)
    {
        const gtsam::Vector3 blue(0, 216.0f/255.0f, 230.0f/255.0f); // Normalized
        const gtsam::Vector3 black(0, 0, 0);
        const float arm_offset = axis_dist_ * 0.5f * SQRT2_INV;

        // Arm positions
        const std::vector<gtsam::Vector3> arm_offsets = {
            gtsam::Vector3(arm_offset, arm_offset, 0),
            gtsam::Vector3(-arm_offset, -arm_offset, 0),
            gtsam::Vector3(-arm_offset, arm_offset, 0),
            gtsam::Vector3(arm_offset, -arm_offset, 0)
        };

        // Draw arms
        glPointSize(2.0f);
        for (const auto& offset : arm_offsets) {
            gtsam::Vector3 end = rot.rotate(offset) + p;
            drawLine(blue, p, end);
        }

        // Draw coordinate frame
        drawFrame(p, rot);

        // Draw propeller circles
        for (const auto& offset : arm_offsets) {
            gtsam::Vector3 center = rot.rotate(offset) + p;
            drawCircle(black, arm_offset*0.8, center, rot);
        }

        // Draw propeller centers
        glColor3f(0, 0, 0);
        glPointSize(3.0f);
        glBegin(GL_POINTS);
        for (const auto& offset : arm_offsets) {
            gtsam::Vector3 center = rot.rotate(offset) + p;
            glVertex3f(center[0], center[1], center[2]);
        }
        glEnd();
    }

    void UI::drawCircle(const gtsam::Vector3& color, float r, const gtsam::Vector3& center, const gtsam::Rot3& rot)
    {
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

    void UI::drawLine(const gtsam::Vector3& color, const gtsam::Vector3& begin, const gtsam::Vector3& end)
    {
        glBegin(GL_LINES);
        glColor3f(color[0], color[1], color[2]);
        glVertex3d(begin[0], begin[1], begin[2]);
        glVertex3d(end[0], end[1], end[2]);
        glEnd();
    }

    void UI::drawLidarCloud(Features& features)
    {
        glBegin(GL_POINTS);
        for (int idx = 0; idx < features.size(); ++idx) {
            if (features[idx].type == PointType::L_VIS) {
                glPointSize(5.0f);
                glColor3f(0.1f, 0.2f, 0.7f);
            } else if (features[idx].type == PointType::L_NONV) {
                glPointSize(3.0f);
                glColor3f(0.1f, 0.8f, 0.7f);
            }
            glVertex3f(features[idx].x, features[idx].y, features[idx].z);
        }
        glEnd();
    }

    void UI::drawFrame(const gtsam::Vector3& p, const gtsam::Rot3& rot)
    {
        const float axis_length = 0.1f;
        const std::vector<std::pair<gtsam::Vector3, gtsam::Vector3>> axes = {
            {gtsam::Vector3(1.0f, 0, 0), gtsam::Vector3(axis_length, 0, 0)}, // Normalized colors
            {gtsam::Vector3(0, 1.0f, 0), gtsam::Vector3(0, axis_length, 0)},
            {gtsam::Vector3(0, 0, 1.0f), gtsam::Vector3(0, 0, axis_length)}
        };

        for (const auto& axis : axes) {
            gtsam::Vector3 end = rot.rotate(axis.second) + p;
            drawLine(axis.first, p, end);
        }
    }

    void UI::renderHistoryTrj()
    {
        if (pangolin::ShouldQuit()) return;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        trj_.push_back(state_);

        d_cam_.Activate(*s_cam_);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());

        // Draw trajectory
        for (size_t i = 0; i < trj_.size() - 1; ++i) {
            drawLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].p, trj_[i + 1].p);
        }

        drawQuadrotor(state_.p, state_.rot);
        renderPanel();

        pangolin::FinishFrame();
        usleep(100);
    }

    std::vector<Point3D> UI::generateSpherePoints(float radius, int numTheta, int numPhi) const
    {
        std::vector<Point3D> points;
        points.reserve(numTheta * numPhi);

        for (int i = 0; i < numTheta; ++i) {
            float theta = 2.0f * M_PI * i / numTheta;
            for (int j = 0; j < numPhi; ++j) {
                float phi = M_PI * j / numPhi;
                points.push_back({
                    radius * std::sin(phi) * std::cos(theta),
                    radius * std::sin(phi) * std::sin(theta),
                    radius * std::cos(phi)
                });
            }
        }
        return points;
    }

    gtsam::Vector3 UI::getObs1() const
    {
        return obstacle_centers_.empty() ? gtsam::Vector3::Zero() : 
               gtsam::Vector3(obstacle_centers_[0].x, obstacle_centers_[0].y, obstacle_centers_[0].z);
    }

    std::vector<gtsam::Vector3> UI::getObstacles() const
    {
        std::vector<gtsam::Vector3> obstacles;
        obstacles.reserve(obstacle_centers_.size());
        for (const auto& center : obstacle_centers_) {
            obstacles.emplace_back(center.x, center.y, center.z);
        }
        return obstacles;
    }

    bool UI::checkCollision(const State& state, const ObsType& type, const gtsam::Vector3& obstacle_center, float r) const
    {
        if(type == ObsType::sphere)
        {
            float obs_distance = (state.p - obstacle_center).norm();
            if (obs_distance >= r) 
            {
                return false;
            }
        // std::cout << "Collision detected: " << obs_distance << std::endl;
        // std::cout << "State p: " << state.p.transpose() << std::endl;
        // std::cout << "Obstacle center: " << obstacle_center.transpose() << std::endl;
        }
        else if(type == ObsType::cylinder)
        {
            gtsam::Matrix3 _E12;
            _E12 << 1,0,0, 0,1,0, 0,0,0;
            float obs_distance = (_E12 * state.p - _E12 * obstacle_center).norm();
            if (obs_distance >= r) 
            {
                return false;
            }
        }
        else
        {
            std::cout << "Unknown obstacles" << std::endl;
        }
        return true;
    }

    void UI::renderHistoryOpt(State& state, std::vector<State>& pred_trj, 
                            boost::optional<gtsam::Vector3&> err, 
                            boost::optional<Features&> features,
                            boost::optional<gtsam::Vector3&> vicon_measurement,
                            boost::optional<gtsam::Vector3&> rot_err,
                            boost::optional<std::vector<State>&> ref_trj,
                            boost::optional<float&> opt_cost,
                            boost::optional<std::vector<Obstacle>&> obstacle_centers)
    {
        clock_ += delta_t_;
        state_.p = state.p;
        state_.rot = state.rot;
        state_.v = state.v;

        if (opt_cost) {
            opt_cost_ = *opt_cost;
        }

        // Record data
        if (rot_err) {
            record_info_ << state_.p[0] << " " << state_.p[1] << " " << state_.p[2] << " "
                       << (*err)[0] << " " << (*err)[1] << " " << (*err)[2] << " "
                       << state_.thrust_torque[0] << " " << state_.thrust_torque[1] << " "
                       << state_.thrust_torque[2] << " " << state_.thrust_torque[3] << " "
                       << (*rot_err)[0] << " " << (*rot_err)[1] << " " << (*rot_err)[2] << std::endl;
        } else {
            record_info_ << state_.p[0] << " " << state_.p[1] << " " << state_.p[2] << " "
                       << (*err)[0] << " " << (*err)[1] << " " << (*err)[2] << " "
                       << state_.thrust_torque[0] << " " << state_.thrust_torque[1] << " "
                       << state_.thrust_torque[2] << " " << state_.thrust_torque[3] << std::endl;
        }

        if (pangolin::ShouldQuit()) return;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Maintain trajectory history
        if (trj_.size() > HISTORY_TRJ_LENS) {
            trj_.erase(trj_.begin());
        }
        trj_.push_back(state_);

        d_cam_.Activate(*s_cam_);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        
        // Draw reference frame
        drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());

        // Draw reference trajectory
        if (ref_trj) {
            glLineWidth(1);
            for (size_t i = 0; i < ref_trj->size(); ++i) {
                drawTrjPoint((*ref_trj)[i].p);
            }
        }

        // Draw predicted trajectory
        glLineWidth(3);
        for (size_t i = 0; i < pred_trj.size() - 1; ++i) {
            drawLine(gtsam::Vector3(1.0, 0, 0), pred_trj[i].p, pred_trj[i + 1].p);
        }

        // Draw obstacles
        if (obstacle_centers) {
            for (size_t i = 0; i < obstacle_centers->size(); ++i) {
                const auto& obs = obstacle_centers->at(i);
                bool collision = checkCollision(state, obs.obs_type, obs.obs_pos, che_dis_);
                
                glColor3f(collision ? 0.3f : 1.0f, 0.5f, collision ? 0.6f : 0.0f);
                glPointSize(collision ? 3.0f : 1.0f);

                if(obs.obs_type == ObsType::sphere)
                {
                    drawSphere(obs.obs_pos, obs.obs_size, gtsam::Vector3(0.133f, 0.545f, 0.133f), 32, 32);
                }
                else if(obs.obs_type == ObsType::cylinder)
                {
                    drawCylinder(obs.obs_pos, obs.obs_size, 1.3f, gtsam::Vector3(0.0f, 0.0f, 0.502f), 32);
                }

                // Check predicted trajectory for collisions
                for (const auto& pred_state : pred_trj) {
                    if (checkCollision(pred_state, obs.obs_type, obs.obs_pos, che_dis_)) {
                        drawCollisionPoint(pred_state.p);
                    }
                }
            }
        }

        // Draw history trajectory
        float trajectory_length = 0.0f;
        glLineWidth(3);
        for (int i = trj_.size() - 1; i > 0; --i) {
            trajectory_length += (trj_[i].p - trj_[i - 1].p).norm();
            if (trajectory_length >= trj_len_max_) break;
            drawLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].p, trj_[i - 1].p);
        }

        // Draw quadrotor
        drawQuadrotor(state_.p, state_.rot);

        // Draw features if available
        if (features) {
            drawLidarCloud(*features);
        }

        // Draw vicon measurement if available
        if (vicon_measurement) 
        {
            drawTrjPoint(*vicon_measurement, 10.0f, gtsam::Vector3(0.6, 0.2, 0.5));
        }
        
        renderPanel();
        pangolin::FinishFrame();
        usleep(1000);
    }

    void UI::renderPanel()
    {
        auto setString = [](StringUI& var, const std::string& value) {
            *var = value;
        };

        auto setPrecision = [](StringUI& var, double value, int precision = 15) {
            std::stringstream ss;
            ss << std::setprecision(precision) << value;
            *var = ss.str();
        };

        setPrecision(str_force_, state_.thrust_torque[0], 6);
        setPrecision(str_M1_, state_.thrust_torque[1]);
        setPrecision(str_M2_, state_.thrust_torque[2]);
        setPrecision(str_M3_, state_.thrust_torque[3]);
        
        setPrecision(str_Quad_x_, state_.p[0], 6);
        setPrecision(str_Quad_y_, state_.p[1], 6);
        setPrecision(str_Quad_z_, state_.p[2], 6);
        
        setPrecision(str_Quad_velx_, state_.v[0], 6);
        setPrecision(str_Quad_vely_, state_.v[1], 6);
        setPrecision(str_Quad_velz_, state_.v[2], 6);
        
        setPrecision(str_timestamp_, clock_, 6);
        setPrecision(str_opt_cost_, opt_cost_, 6);

        for (int i = 0; i < 4; ++i) {
            setPrecision(str_rotor_[i], state_.motor_rpm[i], 6);
        }

        // Calculate average error
        double error_sum = 0.0;
        for (const auto& err : errs_) {
            error_sum += err.squaredNorm();
        }
        double average_error = errs_.empty() ? 0.0 : std::sqrt(error_sum / errs_.size());
        setPrecision(str_AVE_ERR_, average_error, 6);
    }
}