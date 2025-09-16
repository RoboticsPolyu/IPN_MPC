#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <boost/bind.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace QuadrotorSim_SO3
{

    Quadrotor::Quadrotor(void)
    {
          // Existing constructor code
        YAML::Node config               = YAML::LoadFile("../config/quadrotor_TGyro.yaml");
                   g_                   = config["g"].as<double>();
                   mass_                = config["mass"].as<double>();
                   kf_                  = config["k_f"].as<double>();
                   km_                  = config["k_m"].as<double>();
                   motor_time_constant_ = config["time_constant"].as<double>();

        double Ixx = config["Ixx"].as<double>();
        double Iyy = config["Iyy"].as<double>();
        double Izz = config["Izz"].as<double>();
               J_  = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

        prop_radius_ = 0.062;
        arm_length_  = 0.26;
        min_rpm_     = 1200;
        max_rpm_     = 35000;
        esc_factor_  = 1;

        state_.p         = Eigen::Vector3d::Zero();
        state_.v         = Eigen::Vector3d::Zero();
        state_.rot       = gtsam::Rot3::identity();
        state_.body_rate = Eigen::Vector3d::Zero();
        state_.motor_rpm = Eigen::Array4d::Zero();

        input_ = Eigen::Array4d::Zero();

        external_force_.setZero();
        external_torque_.setZero();

          // YAML::Node config   = YAML::LoadFile("../config/quadrotor_TGyro.yaml");
               THRUST_NOISE_MEAN  = config["THRUST_NOISE_MEAN"].as<double>();
               THRUST_NOISE_COV   = config["THRUST_NOISE_COV"].as<double>();
               ANGULAR_SPEED_MEAN = config["ANGULAR_SPEED_MEAN"].as<double>();
               ANGULAR_SPEED_COV  = config["ANGULAR_SPEED_COV"].as<double>();
        double DRAG_FORCE_X       = config["DRAG_FORCE_X"].as<double>();
        double DRAG_FORCE_Y       = config["DRAG_FORCE_Y"].as<double>();
        double DRAG_FORCE_Z       = config["DRAG_FORCE_Z"].as<double>();
        float  trj_len_max        = config["TRJ_LEN_MAX"].as<double>();
               obs_num_           = config["OBS_NUM"].as<uint16_t>();
               cylinder_num_      = config["CYLINDER_NUM"].as<uint16_t>();
               sphere_radius_     = config["OBS1_RADIUS"].as<double>();
               quad_size_         = config["UAV_SIZE"].as<double>();
        uint16_t static_obs_num_  = config["STATIC_OBS_NUM"].as<uint16_t>();

          // double SAFE_D       = config["SAFE_D"].as<double>();
        drag_force_params_ = Eigen::Vector3d(DRAG_FORCE_X, DRAG_FORCE_Y, DRAG_FORCE_Z);
        obstacles_.resize(obs_num_ + cylinder_num_);

        ui_ptr = std::make_shared<UI>(trj_len_max, obs_num_, sphere_radius_);
        static_obstacles_.clear();  // Clear previous obstacles

        const float circleRadius = 1.5f;
        const float maxRadius    = 0.20f;
        const float minRadius    = 0.03f;
        const float centerZ      = 1.0f;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> angleDist(M_PI/2, 3*M_PI/2);  // Left half-circle
        std::uniform_real_distribution<double> radiusDist(minRadius, maxRadius);

        for (int i = 0; i < static_obs_num_; ++i) {
            Obstacle obstacle;
            bool collision;
            int   attempts        = 0;
            const int maxAttempts = 100;  // Prevent infinite loops

            do {
                       collision = false;
                double theta     = angleDist(gen);
                double r         = radiusDist(gen);
                double x         = circleRadius * std::cos(theta);
                double y         = circleRadius * std::sin(theta);

                obstacle.obs_pos  = gtsam::Vector3(x, y, centerZ);
                obstacle.obs_vel  = gtsam::Vector3::Zero();
                obstacle.obs_size = r;

                  // Check collision with existing obstacles
                for (const auto& existing : static_obstacles_) {
                    double dx       = existing.obs_pos.x() - obstacle.obs_pos.x();
                    double dy       = existing.obs_pos.y() - obstacle.obs_pos.y();
                    double dz       = existing.obs_pos.z() - obstacle.obs_pos.z();
                    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (distance < (existing.obs_size + obstacle.obs_size)) {
                        collision = true;
                        break;
                    }
                }

                if (++attempts >= maxAttempts) {
                    std::cerr << "Warning: Max attempts reached. Skipping obstacle." << std::endl;
                    break;
                }
            } while (collision);

            if (!collision) {
                static_obstacles_.push_back(obstacle);
            }
        }
    }

    Quadrotor::Quadrotor(const std::string & yaml_file)
    {
          // Existing constructor code
        YAML::Node config               = YAML::LoadFile(yaml_file);
                   g_                   = config["g"].as<double>();
                   mass_                = config["mass"].as<double>();
                   kf_                  = config["k_f"].as<double>();
                   km_                  = config["k_m"].as<double>();
                   motor_time_constant_ = config["time_constant"].as<double>();

        double Ixx = config["Ixx"].as<double>();
        double Iyy = config["Iyy"].as<double>();
        double Izz = config["Izz"].as<double>();
               J_  = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

        prop_radius_ = 0.062;
        arm_length_  = 0.26;
        min_rpm_     = 1200;
        max_rpm_     = 35000;
        esc_factor_  = 1;

        state_.p         = Eigen::Vector3d::Zero();
        state_.v         = Eigen::Vector3d::Zero();
        state_.rot       = gtsam::Rot3::identity();
        state_.body_rate = Eigen::Vector3d::Zero();
        state_.motor_rpm = Eigen::Array4d::Zero();

        input_ = Eigen::Array4d::Zero();

        external_force_.setZero();
        external_torque_.setZero();

          // YAML::Node config   = YAML::LoadFile("../config/quadrotor_TGyro.yaml");
               THRUST_NOISE_MEAN  = config["THRUST_NOISE_MEAN"].as<double>();
               THRUST_NOISE_COV   = config["THRUST_NOISE_COV"].as<double>();
               ANGULAR_SPEED_MEAN = config["ANGULAR_SPEED_MEAN"].as<double>();
               ANGULAR_SPEED_COV  = config["ANGULAR_SPEED_COV"].as<double>();
        double DRAG_FORCE_X       = config["DRAG_FORCE_X"].as<double>();
        double DRAG_FORCE_Y       = config["DRAG_FORCE_Y"].as<double>();
        double DRAG_FORCE_Z       = config["DRAG_FORCE_Z"].as<double>();
        float  trj_len_max        = config["TRJ_LEN_MAX"].as<double>();
               obs_num_           = config["OBS_NUM"].as<uint16_t>();
               cylinder_num_      = config["CYLINDER_NUM"].as<uint16_t>();
               sphere_radius_     = config["OBS1_RADIUS"].as<double>();
               quad_size_         = config["UAV_SIZE"].as<double>();
        uint16_t static_obs_num_  = config["STATIC_OBS_NUM"].as<uint16_t>();
          // double SAFE_D       = config["SAFE_D"].as<double>();
        drag_force_params_ = Eigen::Vector3d(DRAG_FORCE_X, DRAG_FORCE_Y, DRAG_FORCE_Z);
        obstacles_.resize(obs_num_ + cylinder_num_);

        ui_ptr = std::make_shared<UI>(trj_len_max, obs_num_, sphere_radius_ + quad_size_/2.0);
        static_obstacles_.clear();  // Clear previous obstacles

        const float circleRadius = 1.5f;
        const float maxRadius    = 0.20f;
        const float minRadius    = 0.03f;
        const float centerZ      = 1.0f;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> angleDist(M_PI/2, 3*M_PI/2);  // Left half-circle
        std::uniform_real_distribution<double> radiusDist(minRadius, maxRadius);

        for (int i = 0; i < static_obs_num_; ++i) {
            Obstacle obstacle;
            bool collision;
            int   attempts        = 0;
            const int maxAttempts = 100;  // Prevent infinite loops

            do {
                       collision = false;
                double theta     = angleDist(gen);
                double r         = radiusDist(gen);
                double x         = circleRadius * std::cos(theta);
                double y         = circleRadius * std::sin(theta);

                obstacle.obs_pos  = gtsam::Vector3(x, y, centerZ);
                obstacle.obs_vel  = gtsam::Vector3::Zero();
                obstacle.obs_size = r;

                  // Check collision with existing obstacles
                for (const auto& existing : static_obstacles_) {
                    double dx       = existing.obs_pos.x() - obstacle.obs_pos.x();
                    double dy       = existing.obs_pos.y() - obstacle.obs_pos.y();
                    double dz       = existing.obs_pos.z() - obstacle.obs_pos.z();
                    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (distance < (existing.obs_size + obstacle.obs_size)) {
                        collision = true;
                        break;
                    }
                }

                if (++attempts >= maxAttempts) {
                    std::cerr << "Warning: Max attempts reached. Skipping obstacle." << std::endl;
                    break;
                }
            } while (collision);

            if (!collision) {
                static_obstacles_.push_back(obstacle);
            }
        }
    }

    void Quadrotor::step(double dt)
    {
        State predicted_state_;

        Eigen::Vector3d vnorm;
        Eigen::Array4d motor_rpm_sq;

        motor_rpm_sq = state_.motor_rpm.array().square();

        double thrust = kf_ * motor_rpm_sq.sum();

        Eigen::Vector3d torque;
        torque(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
        torque(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
        torque(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) - motor_rpm_sq(3));

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = - state_.rot.matrix() * Eigen::Matrix3d(drag_force_params_.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot      = - Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_
                                     + external_force_ / mass_ + drag_force;

        Eigen::Vector3d p_dot = state_.v;

          // J* body_rate_dot = torque - J.cross(J* body_rate)
        Eigen::Vector3d body_rate_dot = J_.inverse() * (torque - state_.body_rate.cross(J_ * state_.body_rate) + external_torque_);

          // Predict state
        predicted_state_.p = state_.p + p_dot * dt;
        predicted_state_.v = state_.v + v_dot * dt;
                                     
          // predicted_state_.motor_rpm = state_.motor_rpm + (motor_rpm - state_.motor_rpm) / motor_time_constant_;
        predicted_state_.rot       = state_.rot * gtsam::Rot3::Expmap(state_.body_rate * dt);
        predicted_state_.body_rate = state_.body_rate + body_rate_dot * dt;

        state_ = predicted_state_;
          // Don't go below zero, simulate floor
        if (state_.p(2) < 0.0 && state_.v(2) < 0)
        {
            state_.p(2) = 0;
            state_.v(2) = 0;
        }
    }

    void Quadrotor::operator()(const Quadrotor::stateType &x, Quadrotor::stateType &dxdt, const double t)
    {
        State est_state;
        Eigen::Matrix3d cur_rotm;
        for (int i = 0; i < 3; i++)
        {
            est_state.p(i)         = x[0 + i];
            est_state.v(i)         = x[3 + i];
            cur_rotm(i, 0)         = x[6 + i];
            cur_rotm(i, 1)         = x[9 + i];
            cur_rotm(i, 2)         = x[12 + i];
            est_state.body_rate(i) = x[15 + i];
        }

          // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
        Eigen::Matrix3d P = llt.matrixL();
        Eigen::Matrix3d R = cur_rotm * P.inverse();

        est_state.rot = gtsam::Rot3(R);

        Eigen::Vector3d vnorm;

        std::normal_distribution<double> thrust_noise(THRUST_NOISE_MEAN, THRUST_NOISE_COV);
        double at_noise = thrust_noise(generator_);

        double thrust = x[18] + at_noise;  // cur force

        Eigen::Vector3d torque(x[19], x[20], x[21]);  // cur torque

        vnorm = est_state.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = est_state.rot.matrix() * Eigen::Matrix3d(drag_force_params_.asDiagonal()) * est_state.rot.matrix().transpose() * est_state.v;
        
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + est_state.rot.rotate(gtsam::Vector3(0, 0, thrust / mass_)) +
                                external_force_ / mass_ + drag_force / mass_;

        Eigen::Vector3d p_dot = est_state.v;

        Eigen::Matrix3d r_dot = est_state.rot.matrix() * gtsam::skewSymmetric(est_state.body_rate);

          // J* body_rate_dot = torque - J.cross(J* body_rate)
        Eigen::Vector3d body_rate_dot = J_.inverse() * (torque - est_state.body_rate.cross(J_ * est_state.body_rate) + external_torque_);

        for (int i = 0; i < 3; i++)
        {
            dxdt[0 + i]  = p_dot(i);
            dxdt[3 + i]  = v_dot(i);
            dxdt[6 + i]  = r_dot(i, 0);
            dxdt[9 + i]  = r_dot(i, 1);
            dxdt[12 + i] = r_dot(i, 2);
            dxdt[15 + i] = body_rate_dot(i);
        }

        for (int i = 0; i < 4; i++)
        {
            dxdt[18 + i] = (thrust_torque_[i] - state_.thrust_torque[i]) / motor_time_constant_;
        }

        for (int i = 0; i < 22; ++i)
        {
            if (std::isnan(dxdt[i]))
            {
                dxdt[i] = 0;
                std::cout << "nan apply to 0 for " << i << std::endl;
            }
        }
    }

    void Quadrotor::stepODE(double dt, gtsam::Vector4 fm)
    {
        stateType x;
        Eigen::Matrix3d r = state_.rot.matrix();
        for (int i = 0; i < 3; i++)
        {
            x[0 + i]  = state_.p(i);
            x[3 + i]  = state_.v(i);
            x[6 + i]  = r(i, 0);
            x[9 + i]  = r(i, 1);
            x[12 + i] = r(i, 2);
            x[15 + i] = state_.body_rate(i);
        }

          // float sin_force = 0.1* sin()
          // state_.thrust_torque[0] = state_.thrust_torque[0];

        for (int i = 0; i < 4; i++)
        {

            x[18 + i] = state_.thrust_torque[i];
        }

        thrust_torque_ = fm;  // control at future dt.
        integrate(boost::ref(*this), x, 0.0, dt, dt);

        Eigen::Matrix3d cur_rotm;
        for (int i = 0; i < 3; i++)
        {
            state_.p(i)         = x[0 + i];
            state_.v(i)         = x[3 + i];
            cur_rotm(i, 0)      = x[6 + i];
            cur_rotm(i, 1)      = x[9 + i];
            cur_rotm(i, 2)      = x[12 + i];
            state_.body_rate(i) = x[15 + i];
        }

        for (int i = 0; i < 4; i++)
        {
            state_.thrust_torque[i] = x[18 + i];
        }

          // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
        Eigen::Matrix3d P = llt.matrixL();
        Eigen::Matrix3d R = cur_rotm * P.inverse();

        state_.rot = gtsam::Rot3(R);

        std::normal_distribution<double> angular_speed_noise(ANGULAR_SPEED_MEAN, ANGULAR_SPEED_COV);
        gtsam::Vector3 ome_noise = gtsam::Vector3(angular_speed_noise(generator_), angular_speed_noise(generator_), 0.01* angular_speed_noise(generator_));

        state_.p         = state_.p;
        state_.v         = state_.v;
        state_.body_rate = state_.body_rate + ome_noise;

          // printCurState();
    }

    Eigen::Matrix4d Quadrotor::ComputeEffectivenessMatrix()
    {
        Eigen::Matrix4d effectivenessMatrix;

          // effectivenessMatrix << 
          //     kf_, kf_, kf_, kf_,
          //     0, 0, arm_length_ * kf_, - arm_length_ * kf_,
          //     - arm_length_ * kf_, arm_length_ * kf_, 0, 0,
          //     km_, km_, -km_, -km_;
        double dx0 = 0.10f;
        double dx1 = 0.10f;

        effectivenessMatrix << kf_, kf_, kf_, kf_,
              dx0 * kf_, - dx0 * kf_, - dx0 * kf_,   dx0 * kf_,
            - dx1 * kf_, - dx1 * kf_,   dx0 * kf_,   dx0 * kf_,
              km_ * kf_, - km_ * kf_,   km_ * kf_, - km_ * kf_;
        
        std::cout << "effectivenessMatrix: " << effectivenessMatrix / kf_ << std::endl;
        return effectivenessMatrix;
    }

    Eigen::Vector4d Quadrotor::CumputeRotorsVel()
    {
        effectiveness_ = ComputeEffectivenessMatrix();

        Eigen::Vector4d thrust;
                        thrust   = effectiveness_.inverse()* thrust_torque_;
        Eigen::Vector4d identity = Eigen::Vector4d::Identity();

          // esc_factor* Actuator_Ouput^2 + (1 - esc_factor)* Actuator_Output - rotor_thrust = 0
        
        Eigen::Vector4d actuator_output;
        double a = esc_factor_;
        double b = 1 - esc_factor_;
    
        actuator_output = (- b * identity + (Eigen::Vector4d)(b * b * identity - 4 * a * (- thrust)).array().sqrt()) / (2 * a);
        input_          = actuator_output;
        
        return actuator_output;
    }

    Eigen::Vector4d Quadrotor::InvCumputeRotorsVel(Eigen::Vector4d rotor_speed)
    {
        effectiveness_ = ComputeEffectivenessMatrix();

        Eigen::Vector4d actuator_output; 

        Eigen::Vector4d thrust_torque;
        actuator_output = esc_factor_* rotor_speed.cwiseAbs2() + (1- esc_factor_)* rotor_speed;

        thrust_torque = effectiveness_* actuator_output;
        return thrust_torque;

          // esc_factor* Actuator_Ouput^2 + (1 - esc_factor)* Actuator_Output - rotor_thrust = 0
    }

    void Quadrotor::printCurState()
    {
        Color::Modifier red(Color::FG_RED);
        Color::Modifier def(Color::FG_DEFAULT);
        Color::Modifier green(Color::FG_GREEN);
        std::cout << red;
        std::cout << "Cur Position: "  << state_.p.transpose() << std::endl;
        std::cout << "Cur Rotation: "  << Rot3::Logmap(state_.rot).transpose() << std::endl;
        std::cout << "Cur Velocity: "  << state_.v.transpose() << std::endl;
        std::cout << "Cur body_rate: " << state_.body_rate.transpose() << std::endl;
        std::cout << "Cur ForceMom: "  << state_.thrust_torque.transpose() << std::endl;
        std::cout << def;
    }
    void Quadrotor::setInput(gtsam::Vector4 thrust_torque)
    {
        thrust_torque_ = thrust_torque;
    }


    void Quadrotor::setInput(double u1, double u2, double u3, double u4)
    {
        input_(0) = u1;
        input_(1) = u2;
        input_(2) = u3;
        input_(3) = u4;
        for (int i = 0; i < 4; i++)
        {
            if (std::isnan(input_(i)))
            {
                input_(i) = (max_rpm_ + min_rpm_) / 2;
                std::cout << "NAN input ";
            }
            if (input_(i) > max_rpm_)
                input_(i) = max_rpm_;
            else if (input_(i) < min_rpm_)
                input_(i) = min_rpm_;
        }
        state_.motor_rpm << u1, u2, u3, u4;
    }

    const State &Quadrotor::getState(void) const
    {
        return state_;
    }
    void Quadrotor::setState(const State &state)
    {
        state_ = state;
        // state_.p             = state.p;
        // state_.v             = state.v;
        // state_.rot           = state.rot;
        // state_.body_rate     = state.body_rate;
        // state_.motor_rpm     = state.motor_rpm;
        // state_.thrust_torque = state.thrust_torque;
        // state_.timestamp     = state.timestamp;
    }

    void Quadrotor::setStatePos(const Eigen::Vector3d &Pos)
    {
        state_.p = Pos;
    }

    double Quadrotor::getMass(void) const
    {
        return mass_;
    }
    void Quadrotor::setMass(double mass)
    {
        mass_ = mass;
    }

    double Quadrotor::getGravity(void) const
    {
        return g_;
    }
    void Quadrotor::setGravity(double g)
    {
        g_ = g;
    }

    const Eigen::Matrix3d &Quadrotor::getInertia(void) const
    {
        return J_;
    }
    void Quadrotor::setInertia(const Eigen::Matrix3d &inertia)
    {
        if (inertia != inertia.transpose())
        {
            std::cerr << "Inertia matrix not symmetric, not setting" << std::endl;
            return;
        }
        J_ = inertia;
    }

    double Quadrotor::getArmLength(void) const
    {
        return arm_length_;
    }
    void Quadrotor::setArmLength(double d)
    {
        if (d <= 0)
        {
            std::cerr << "Arm length <= 0, not setting" << std::endl;
            return;
        }

        arm_length_ = d;
    }

    double Quadrotor::getPropRadius(void) const
    {
        return prop_radius_;
    }
    void Quadrotor::setPropRadius(double r)
    {
        if (r <= 0)
        {
            std::cerr << "Prop radius <= 0, not setting" << std::endl;
            return;
        }
        prop_radius_ = r;
    }

    double Quadrotor::getPropellerThrustCoefficient(void) const
    {
        return kf_;
    }
    void Quadrotor::setPropellerThrustCoefficient(double kf)
    {
        if (kf <= 0)
        {
            std::cerr << "Thrust coefficient <= 0, not setting" << std::endl;
            return;
        }

        kf_ = kf;
    }

    double Quadrotor::getPropellerMomentCoefficient(void) const
    {
        return km_;
    }
    void Quadrotor::setPropellerMomentCoefficient(double km)
    {
        if (km <= 0)
        {
            std::cerr << "Moment coefficient <= 0, not setting" << std::endl;
            return;
        }

        km_ = km;
    }

    double Quadrotor::getMotorTimeConstant(void) const
    {
        return motor_time_constant_;
    }
    void Quadrotor::setMotorTimeConstant(double k)
    {
        if (k <= 0)
        {
            std::cerr << "Motor time constant <= 0, not setting" << std::endl;
            return;
        }

        motor_time_constant_ = k;
    }

    const Eigen::Vector3d & Quadrotor::getExternalForce(void) const
    {
        return external_force_;
    }
    void Quadrotor::setExternalForce(const Eigen::Vector3d &force)
    {
        external_force_ = force;
    }

    const Eigen::Vector3d &Quadrotor::getExternalMoment(void) const
    {
        return external_torque_;
    }
    void Quadrotor::setExternalMoment(const Eigen::Vector3d &torque)
    {
        external_torque_ = torque;
    }

    double Quadrotor::getMaxRPM(void) const
    {
        return max_rpm_;
    }
    void Quadrotor::setMaxRPM(double max_rpm)
    {
        if (max_rpm <= 0)
        {
            std::cerr << "Max rpm <= 0, not setting" << std::endl;
            return;
        }
        max_rpm_ = max_rpm;
    }

    double Quadrotor::getMinRPM(void) const
    {
        return min_rpm_;
    }
    void Quadrotor::setMinRPM(double min_rpm)
    {
        if (min_rpm < 0)
        {
            std::cerr << "Min rpm < 0, not setting" << std::endl;
            return;
        }
        min_rpm_ = min_rpm;
    }

    Eigen::Vector3d Quadrotor::getAcc() const
    {
        return acc_;
    }
   
    void Quadrotor::renderHistoryOpt(std::vector<State> & pred_trj, boost::optional<gtsam::Vector3&> err,  
                                                                    boost::optional<Features&> features, 
                                                                    boost::optional<gtsam::Vector3&> vicon_measurement, 
                                                                    boost::optional<gtsam::Vector3 &> rot_err, 
                                                                    boost::optional<std::vector<State> &> ref_trj,
                                                                    boost::optional<float &> opt_cost)
    {
        clock_ = clock_ + 0.01f;
        ui_ptr->renderHistoryOpt(state_, pred_trj, err, features, vicon_measurement, rot_err, ref_trj, opt_cost, obstacles_);
          // unsigned int microsecond = 1000000;
          // usleep(0.1 * microsecond);
    }


    gtsam::Vector3 Quadrotor::getObsbyEllipse(uint8_t index) 
    { 
        gtsam::Vector3 point3d{0,0,0};
        if(index >= obs_num_)
        {
            return point3d;
        }
        float a = 1.10;
        float b = 0.50;
        float v = 0.40;
        float z = 1.00;

          // clock_ = 10.0; // static obstacles

        double t = clock_ + index * 2.0 * M_PI / obs_num_ * sqrt(a * a + b * b) / v;  // Spread obstacles evenly over one cycle

        double angle = v * t / sqrt(a * a + b * b);
        double x     = a * cos(angle);
        double y     = b * sin(angle);

          // // rotating
          // double theta = M_PI / 4;
          // double rotatedX = x * cos(theta) - y * sin(theta); 
          // double rotatedY = x * sin(theta) + y * cos(theta);

        point3d[0] = x - a/2;
        point3d[1] = y - b/2;
        point3d[2] = z;
        return point3d;
    }

    bool checkCollision(const Obstacle& a, const Obstacle& b) {
        double dx           = a.obs_pos.x() - b.obs_pos.x();
        double dy           = a.obs_pos.y() - b.obs_pos.y();
        double distance_sq  = dx*dx + dy*dy;
        double min_distance = a.obs_size + b.obs_size;
        return distance_sq < (min_distance * min_distance);
    }

    void Quadrotor::initializeObstacles() {
        const float circleRadius = 1.5f;
        const float maxRadius    = 0.20f;
        const float minRadius    = 0.10f;
        const float centerZ      = 1.0f;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> angleDist(M_PI/2, 3*M_PI/2);
        std::uniform_real_distribution<double> radiusDist(minRadius, maxRadius);
        std::uniform_real_distribution<double> velDist(-0.1, 0.1);

        for (int i = 0; i < static_obs_num_; ++i) {
            Obstacle obstacle;
            bool collision;
            int   attempts        = 0;
            const int maxAttempts = 100;

            do {
                       collision = false;
                double theta     = angleDist(gen);
                double r         = radiusDist(gen);
                double x         = circleRadius * std::cos(theta);
                double y         = circleRadius * std::sin(theta);

                obstacle.obs_pos  = gtsam::Vector3(x, y, centerZ);
                obstacle.obs_vel  = gtsam::Vector3(velDist(gen), velDist(gen), 0.0);
                obstacle.obs_size = r;

                for (const auto& existing : static_obstacles_) {
                    if (checkCollision(obstacle, existing)) {
                        collision = true;
                        break;
                    }
                }

                if (++attempts >= maxAttempts) {
                    std::cerr << "Warning: Max attempts reached. Skipping obstacle." << std::endl;
                    break;
                }
            } while (collision);

            if (!collision) {
                static_obstacles_.push_back(obstacle);
            }
        }
    }

    void Quadrotor::resolveCollisions() {
        const double response_factor = 0.5;   // How much to push apart
        const double min_separation  = 0.01;  // Minimum enforced separation
        
        for (size_t i = 0; i < static_obstacles_.size(); ++i) {
            for (size_t j = i+1; j < static_obstacles_.size(); ++j) {
                Obstacle& a = static_obstacles_[i];
                Obstacle& b = static_obstacles_[j];
                
                if (checkCollision(a, b)) {
                      // Calculate collision normal and overlap
                    gtsam::Vector3 delta    = a.obs_pos - b.obs_pos;
                    double         distance = delta.norm();
                    double         overlap  = (a.obs_size + b.obs_size) - distance;
                    
                    if (distance > 0) {
                        gtsam::Vector3 collision_normal = delta / distance;
                        
                          // Push apart
                        double push       = response_factor * overlap;
                               a.obs_pos += collision_normal * push;
                               b.obs_pos -= collision_normal * push;
                        
                          // Adjust velocities to avoid sticking
                        double dot_product = a.obs_vel.dot(collision_normal) - b.obs_vel.dot(collision_normal);
                        if (dot_product < 0) { // Moving toward each other
                            gtsam::Vector3 impulse    = collision_normal * dot_product;
                                           a.obs_vel -= impulse * 0.5;
                                           b.obs_vel += impulse * 0.5;
                        }
                    }
                }
            }
        }
    }


    template<typename T>
    const T& clamp(const T& value, const T& min, const T& max) {
        return (value < min) ? min: (max < value) ? max: value;
    }

    void Quadrotor::updateObstaclePositions(double dt) {
          // First update all positions
        for (auto& obstacle : static_obstacles_) {
            obstacle.obs_pos += obstacle.obs_vel * dt;
        }
        
          // Then resolve any collisions
        resolveCollisions();
        
          // Add some randomness to movement
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> velChange(-0.02, 0.02);
        
        for (auto& obstacle : static_obstacles_) {
              // Slightly modify velocity
            obstacle.obs_vel.x() += velChange(gen);
            obstacle.obs_vel.y() += velChange(gen);
            
              // Keep velocity within bounds
            for (int i = 0; i < 2; i++) {
                if (obstacle.obs_vel(i) > 0.1) obstacle.obs_vel(i)  = 0.1;
                if (obstacle.obs_vel(i) < -0.1) obstacle.obs_vel(i) = -0.1;
            }
            
              // Boundary checking
            const double maxX = 2.0, minX = -2.0;
            const double maxY = 2.0, minY = -2.0;
            
            if (obstacle.obs_pos.x() > maxX || obstacle.obs_pos.x() < minX) {
                obstacle.obs_vel.x() *= -1;
                obstacle.obs_pos.x()  = clamp(obstacle.obs_pos.x(), minX, maxX);
            }
            if (obstacle.obs_pos.y() > maxY || obstacle.obs_pos.y() < minY) {
                obstacle.obs_vel.y() *= -1;
                obstacle.obs_pos.y()  = clamp(obstacle.obs_pos.y(), minY, maxY);
            }
        }
    }


    Obstacle Quadrotor::getObsbyEllipsev(uint8_t index) 
    { 
        Obstacle obstacle;
        if(index >= obs_num_)
        {
            return obstacle;  // returns default obstacle (zero position and velocity)
        }
        
        if(index > static_obs_num_ - 1)
        {
                             // Parameters
            float a = 1.10;  // semi-major axis
            float b = 0.50;  // semi-minor axis
            float v = 2.40;  // velocity parameter
            float z = 1.00;  // fixed height
            
              // Calculate time parameter with even spacing
            double t = clock_ + (index - static_obs_num_ + 1) * 2.0 * M_PI / (obs_num_ - static_obs_num_) * sqrt(a * a + b * b) / v;
            
              // Position calculation
            double angle = v * t / sqrt(a * a + b * b);
            double x     = a * cos(angle);
            double y     = b * sin(angle);
            
              // Velocity calculation (derivative of position)
            double dx = -a * sin(angle) * (v / sqrt(a * a + b * b));
            double dy = b * cos(angle) * (v / sqrt(a * a + b * b));
            
              // Set obstacle properties
              // obstacle.obs_pos[0] = x - a/2;
              // obstacle.obs_pos[1] = y - b/2;
            obstacle.obs_pos[0] = x;
            obstacle.obs_pos[1] = y;
            obstacle.obs_pos[2] = z;
            obstacle.obs_vel[0] = dx;
            obstacle.obs_vel[1] = dy;
            obstacle.obs_vel[2] = 0;                // no vertical movement
            obstacle.obs_type   = ObsType::cylinder;
            obstacle.obs_size   = sphere_radius_;
            return obstacle;
        }
        else
        {
              // updateObstaclePositions(dt_);
            return static_obstacles_[index];
        }
    }
}
