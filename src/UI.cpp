#include "UI.h"


namespace QuadrotorSim_SO3
{
    UI::UI(float trj_len_max, uint8_t obs_num, double che_dis)
    {
        trj_len_max_ = trj_len_max;
        obs_num_ = obs_num;
        che_dis_ = che_dis;
        
        float radius = 0.20f; int numTheta = 100; int numPhi = 100;
        spherePoints_ = generateSpherePoints(radius, numTheta, numPhi);
        int numPoints = 200; radius = 0.05; float height = 0.50;
        cylinderPoints_ = generatePointsOutsideCylinder(numPoints, radius, height);

        // Initialize obstacle centers (default N=5)
        std::cout << "Obs num is " << obs_num_ << std::endl;
        std::cout << "obstacle centers " << obstacle_centers_.size() << std::endl;
        // obstacle_centers_.resize(obs_num_);

        std::cout << "obstacle centers " << obstacle_centers_.size() << std::endl;

        record_info_.open("../data/record_info.txt");
        clock_ = 0.0f;
        prop_radius_ = 0.062;
        
        // state_.p = Eigen::Vector3d::Zero();
        // state_.v = Eigen::Vector3d::Zero();
        // state_.rot = gtsam::Rot3::identity();
        // state_.body_rate = Eigen::Vector3d::Zero();
        // state_.motor_rpm = Eigen::Array4d::Zero();

        displaySetup();
    }

    void UI::displaySetup()
    {
        axis_dist_ = 0.30;
        propeller_dist_ = 0.10;

        pangolin::CreateWindowAndBind("Model Predictive Control based on FGO", 1600, 800);

        // Define Camera Render Object (for view / scene browsing)
        s_cam = std::make_shared<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(1600, 1600, 800, 800, 800, 800, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0, 1, 0, 0, 0, 0.0, -1.0, 0.0));

        // Choose a sensible left UI Panel width based on the width of 20
        // charectors from the default font.
        const int UI_WIDTH = 20 * pangolin::default_font().MaxWidth();

        d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 1600.0f / 800.0f).SetHandler(new pangolin::Handler3D(*s_cam));

        pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

        str_force_     = std::make_shared<pangolin::Var<std::string>>("ui.Force(N)", "Force");
        str_M1_        = std::make_shared<pangolin::Var<std::string>>("ui.M1(N*m)", "M1");
        str_M2_        = std::make_shared<pangolin::Var<std::string>>("ui.M2(N*m)", "M2");
        str_M3_        = std::make_shared<pangolin::Var<std::string>>("ui.M3(N*m)", "M3");
        str_Quad_x_    = std::make_shared<pangolin::Var<std::string>>("ui.UAVx(m)", "UAVx");
        str_Quad_y_    = std::make_shared<pangolin::Var<std::string>>("ui.UAVy(m)", "UAVy");
        str_Quad_z_    = std::make_shared<pangolin::Var<std::string>>("ui.UAVz(m)", "UAVz");
        str_Quad_velx_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vx(m/s)", "UAV_vx");
        str_Quad_vely_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vy(m/s)", "UAV_vy");
        str_Quad_velz_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vz(m/s)", "UAV_vz");
        str_AVE_ERR_   = std::make_shared<pangolin::Var<std::string>>("ui.AVE_ERR(m)", "AVE_ERR");
        str_timestamp_ = std::make_shared<pangolin::Var<std::string>>("ui.TIMESTAMP(s)", "TIMESTAMP");
        str_opt_cost_  = std::make_shared<pangolin::Var<std::string>>("ui.OPT_COST(s)", "OPT_COST");
        str_rotor_[0]  = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR1(RPM)", "ROTOR1");
        str_rotor_[1]  = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR2(RPM)", "ROTOR2");
        str_rotor_[2]  = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR3(RPM)", "ROTOR3");
        str_rotor_[3]  = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR4(RPM)", "ROTOR4");
    }

    void UI::drawTrjP(gtsam::Vector3 p)
    {
        glPointSize(3.0);
        glBegin(GL_POINTS);
        glColor3f(0.1, 0.2, 0.7);
        glVertex3f(p[0], p[1], p[2]);
        glEnd();
    }

    void UI::drawTrjPColli(gtsam::Vector3 p)
    {
        glPointSize(10.0);
        glBegin(GL_POINTS);
        glColor3f(0.1, 0.2, 0.7);
        glVertex3f(p[0], p[1], p[2]);
        glEnd();
    }

    void UI::drawQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot)
    {   
        gtsam::Vector3 begin;
        gtsam::Vector3 end;
        gtsam::Vector3 blue(0,216,230);
        glPointSize(2.0);
        begin = p;
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(blue, begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(blue, begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(blue, begin, end);
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(blue, begin, end);
        drawFrame(p, rot);

        glColor3f(0, 0, 0);
        glPointSize(3.0);
        glBegin(GL_POINTS);
        for (int i = 0; i < 360; i++)
        {
            float x = sin((float)i / 180 * M_PI) * axis_dist_ / 10 + axis_dist_ / 2 / 1.414;
            float y = cos((float)i / 180 * M_PI) * axis_dist_ / 10 + axis_dist_ / 2 / 1.414;
            gtsam::Vector3 point = rot.rotate(gtsam::Vector3(x, y, 0)) + begin;
            glVertex3f(point[0], point[1], point[2]);
        }
        for (int i = 0; i < 360; i++)
        {
            float x = sin((float)i / 180 * M_PI) * axis_dist_ / 10 + -axis_dist_ / 2 / 1.414;
            float y = cos((float)i / 180 * M_PI) * axis_dist_ / 10 + -axis_dist_ / 2 / 1.414;
            gtsam::Vector3 point = rot.rotate(gtsam::Vector3(x, y, 0)) + begin;
            glVertex3f(point[0], point[1], point[2]);
        }
        for (int i = 0; i < 360; i++)
        {
            float x = sin((float)i / 180 * M_PI) * axis_dist_ / 10 + -axis_dist_ / 2 / 1.414;
            float y = cos((float)i / 180 * M_PI) * axis_dist_ / 10 + axis_dist_ / 2 / 1.414;
            gtsam::Vector3 point = rot.rotate(gtsam::Vector3(x, y, 0)) + begin;
            glVertex3f(point[0], point[1], point[2]);
        }
        for (int i = 0; i < 360; i++)
        {
            float x = sin((float)i / 180 * M_PI) * axis_dist_ / 10 + axis_dist_ / 2 / 1.414;
            float y = cos((float)i / 180 * M_PI) * axis_dist_ / 10 + -axis_dist_ / 2 / 1.414;
            gtsam::Vector3 point = rot.rotate(gtsam::Vector3(x, y, 0)) + begin;
            glVertex3f(point[0], point[1], point[2]);
        }
        glEnd();

        drawCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
        drawCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
        drawCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
        drawCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
            
    }

    void UI::drawCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot)
    {
    }

    void UI::drawLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end)
    {
        glBegin(GL_LINES);
        glColor3f(color(0), color(1), color(2));
        glVertex3d(begin(0), begin(1), begin(2));
        glVertex3d(end(0), end(1), end(2));
        glEnd();
    }

    void UI::drawLidarCloud(Features &features)
    {
        glBegin(GL_POINTS);

        for (int idx = 0; idx < features.size(); idx++)
        {
            // gtsam::Vector3 l_body_body(features[idx].x, features[idx].y, features[idx].z);
            // gtsam::Vector3 l_body_w = state_.rot.rotate(l_body_body) + state_.p;
           
            if(features[idx].type == PointType::L_VIS)
            {
                glPointSize(5.0);
                glColor3f(0.1, 0.2, 0.7);
            }
            else if(features[idx].type == PointType::L_NONV)
            {
                glPointSize(3.0);
                glColor3f(0.1, 0.8, 0.7);
            }
            glVertex3f(features[idx].x, features[idx].y, features[idx].z);
        }

        glEnd();
    }

    void UI::drawFrame(gtsam::Vector3 p, gtsam::Rot3 rot)
    {
        gtsam::Vector3 begin = p;
        gtsam::Vector3 end;
        end = rot.rotate(gtsam::Vector3(0.1, 0, 0)) + begin;
        drawLine(gtsam::Vector3(5, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0.1, 0)) + begin;
        drawLine(gtsam::Vector3(0, 5, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0, 0.1)) + begin;
        drawLine(gtsam::Vector3(0, 0, 5), begin, end);
    }

    void UI::renderHistoryTrj()
    {
        if (!pangolin::ShouldQuit())
        {
            // Clear screen and activate view to renderHistoryTrj into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            trj_.push_back(state_);

            d_cam.Activate(*s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(2);
            drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());
            for (int i = 0; i < trj_.size() - 1; i++)
            {
                drawLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].p, trj_[i + 1].p);
            }
            drawQuadrotor(state_.p, state_.rot);

            renderPanel();

            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(100); // 0.1ms
        }
    }

    // Function to generate points on the surface of a sphere
    std::vector<Point3D> UI::generateSpherePoints(float radius, int numTheta, int numPhi) 
    {
        std::vector<Point3D> points;
        for (int i = 0; i < numTheta; ++i) {
            float theta = 2.0f * M_PI * i / numTheta;
            for (int j = 0; j < numPhi; ++j) {
                float phi = M_PI * j / numPhi;
                Point3D point;
                point.x = radius * std::sin(phi) * std::cos(theta);
                point.y = radius * std::sin(phi) * std::sin(theta);
                point.z = radius * std::cos(phi);
                points.push_back(point);
            }
        }
        return points;
    }

    std::vector<Point3D> UI::generatePointsOutsideCylinder(int numTheta, float radius, float height) 
    {
        std::vector<Point3D> points;
        std::srand(std::time(0)); // Seed for random number generation

        int height_num = 1000;
        for(int h = 0; h < height_num; h++)
        {
            for (int i = 0; i < numTheta; ++i) {
                float theta = 2.0f * M_PI * i / numTheta;
                Point3D point;
                point.x = radius * std::cos(theta);
                point.y = radius * std::sin(theta);
                point.z = height / height_num * h;
                points.push_back(point);
            }
        }

        return points;
    }
    gtsam::Vector3 UI::getObs1()
    {
        return gtsam::Vector3(sphereCenter_.x, sphereCenter_.y, sphereCenter_.z);
    }
    
    std::vector<gtsam::Vector3> UI::getObsN()
    {
        std::vector<gtsam::Vector3> obstacles;
        obstacles.reserve(obstacle_centers_.size());
        for (const auto& center : obstacle_centers_) {
            obstacles.emplace_back(center.x, center.y, center.z);
        }
        return obstacles;
    }   

    bool UI::checkCollision(const State &state, const gtsam::Vector3& obstacle_center)
    {
        float obs_distance = (state.p - obstacle_center).norm();
        if(obs_distance >= che_dis_)
        {
            return false;
        }
        else
        {
            std::cout << " Happening collision: " << obs_distance << std::endl;
            std::cout << " state p: " << state.p.transpose() << std::endl;
            std::cout << " obstacle_center: " << obstacle_center.transpose() << std::endl;
            return true;
        }
    }

    // Modified renderHistoryOpt to plot N obstacles
    void UI::renderHistoryOpt(State& state, std::vector<State> &pred_trj, boost::optional<gtsam::Vector3 &> err, boost::optional<Features &> features, 
        boost::optional<gtsam::Vector3&> vicon_measurement, boost::optional<gtsam::Vector3 &> rot_err, boost::optional<std::vector<State> &> ref_trj,
        boost::optional<float &> opt_cost, boost::optional<std::vector<gtsam::Vector3> &> obstacle_centers)
    {
        clock_ = clock_ + delta_t;
        state_.p = state.p;
        state_.v = state.v;
        gtsam::Vector3 error = *err;
        
        if(opt_cost)
        {
            opt_cost_ = *opt_cost;
        }
        if(rot_err)
        {
            gtsam::Vector3 rot_error = *rot_err;
            record_info_ << state_.p[0] << " " << state_.p[1] << " " << state_.p[2] << " " << error[0] << " " << error[1] << " " << error[2] << " " << state_.thrust_torque[0] << " " 
                << state_.thrust_torque[1] << " " << state_.thrust_torque[2] << " " << state_.thrust_torque[3] << " " 
                << rot_error[0] << " " << rot_error[1] << " " << rot_error[2] << std::endl;
        }
        else
        {
            record_info_ << state_.p[0] << " " << state_.p[1] << " " << state_.p[2] << " " << error[0] << " " << error[1] << " " << error[2] << " " << state_.thrust_torque[0] << " " << state_.thrust_torque[1] << " " << state_.thrust_torque[2] << " " << state_.thrust_torque[3] << std::endl;
        }

        if (!pangolin::ShouldQuit())
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if (trj_.size() > HISTORY_TRJ_LENS)
            {
                trj_.erase(trj_.begin());
            }
            
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(3);
            drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());
 
            trj_.push_back(state_);
            d_cam.Activate(*s_cam);
            
            std::cout << "obstacle centers " << obstacle_centers->size() << std::endl;
            
            glLineWidth(1);
            // Ref Trajectory
            for (int i = 0; i < ref_trj->size() - 1; i++)
            {
                drawTrjP((*ref_trj)[i].p);
            }

            glLineWidth(3);
            // Predicted Trajectory
            for (int i = 0; i < pred_trj.size() - 1; i++)
            {
                drawLine(gtsam::Vector3(1.0, 0, 0), pred_trj[i].p, pred_trj[i + 1].p);
            }

            // Plot all obstacles
            for (int i = 0; i <  obstacle_centers->size(); i++) 
            {
                gtsam::Vector3 center = obstacle_centers->at(i);
                // std::cout << "center.x: " << center.x() << ", center y: " << center.y() << ", center.z: " << center.z() << std::endl;
                if(checkCollision(state, center))
                {
                    glColor3f(0.3, 0.5, 0.6); // collision
                    glPointSize(3.0);
                }
                else
                {
                    glColor3f(1.0, 0.5, 0.0); // Orange color for obstacles
                    glPointSize(1.0);
                }
                glBegin(GL_POINTS);
                for (const auto& point : spherePoints_) {
                    glVertex3f(point.x + center.x(), point.y + center.y(), point.z + center.z());
                }
                glEnd();
                
                for (int i = 0; i < pred_trj.size(); i++)
                {
                    State _state;
                    _state.p = pred_trj[i].p;
                    if(checkCollision(_state, center))
                    {
                        drawTrjPColli(pred_trj[i].p);
                    }
                }
            }
            
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(3);
            drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());
            // Old Trajectory
            float trj_ls = 0;
            for (int i = trj_.size() - 1; i > 0; i--)
            {
                trj_ls += std::abs(( trj_[i].p - trj_[i - 1].p).norm());
                if(trj_ls >= trj_len_max_)
                {
                    break;
                }
                drawLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].p, trj_[i - 1].p);
            }

            drawQuadrotor(state_.p, state_.rot);

            if (errs_.size() >= ERRS_LENS)
            {
                errs_.erase(errs_.begin());
            }
            errs_.push_back(error);

            if(features)
            {
                Features f = *features;
                drawLidarCloud(f); 
            }

            if(vicon_measurement)
            {
                glColor3f(0.6, 0.2, 0.5);
                glPointSize(10.0);
                glBegin(GL_POINTS);
                glVertex3f(vicon_measurement->x(), vicon_measurement->y(), vicon_measurement->z());
                glEnd();
            }
            renderPanel();

            pangolin::FinishFrame();
            usleep(1000);
        }
    }

    void UI::renderPanel()
    {
        std::string temp_str = std::to_string(state_.thrust_torque[0]);
        *str_force_ = temp_str;
        std::stringstream ss;
        ss << std::setprecision(15) << state_.thrust_torque[1];
        *str_M1_ = ss.str();
        ss << std::setprecision(15) << state_.thrust_torque[2];
        *str_M2_ = ss.str();
        ss << std::setprecision(15) << state_.thrust_torque[3];
        *str_M3_ = ss.str();
        temp_str = std::to_string(state_.p[0]);
        *str_Quad_x_ = temp_str;
        temp_str = std::to_string(state_.p[1]);
        *str_Quad_y_ = temp_str;
        temp_str = std::to_string(state_.p[2]);
        *str_Quad_z_ = temp_str;
        temp_str = std::to_string(state_.v[0]);
        *str_Quad_velx_ = temp_str;
        temp_str = std::to_string(state_.v[1]);
        *str_Quad_vely_ = temp_str;
        temp_str = std::to_string(state_.v[2]);
        *str_Quad_velz_ = temp_str;
        temp_str = std::to_string(clock_);
        *str_timestamp_ = temp_str;
        temp_str = std::to_string(input_[0]);
        *str_rotor_[0] = temp_str;
        temp_str = std::to_string(input_[1]);
        *str_rotor_[1] = temp_str;
        temp_str = std::to_string(input_[2]);
        *str_rotor_[2] = temp_str;
        temp_str = std::to_string(input_[3]);
        *str_rotor_[3] = temp_str;
        temp_str = std::to_string(opt_cost_);
        *str_opt_cost_ = temp_str;
        double err_sum = 0;
        for (uint j = 0; j < errs_.size(); j++)
        {
            err_sum += errs_[j].transpose() * errs_[j];
        }
        double ave_err = std::sqrt(err_sum / errs_.size());
        temp_str = std::to_string(ave_err);
        *str_AVE_ERR_ = temp_str;
    }

}