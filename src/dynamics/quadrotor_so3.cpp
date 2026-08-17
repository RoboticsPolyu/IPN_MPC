#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <filesystem>
#include <iostream>
#include <ipn_mpc/common/logging.h>
#include <ipn_mpc/dynamics/quadrotor_so3.h>
#include <yaml-cpp/yaml.h>

namespace QuadrotorSim_SO3 {
namespace {

std::string trajectoryRecordPath(const std::string& simulator_config_path) {
    return (std::filesystem::path(simulator_config_path).parent_path() / "../log/record_info.txt")
        .lexically_normal()
        .string();
}

} // namespace

Quadrotor::Quadrotor(void) {
    // Existing constructor code
    YAML::Node config = YAML::LoadFile("../config/quadrotor_thrust_gyro.yaml");
    g_ = config["g"].as<double>();
    mass_ = config["mass"].as<double>();
    kf_ = config["k_f"].as<double>();
    km_ = config["k_m"].as<double>();
    motor_time_constant_ = config["time_constant"].as<double>();

    double ixx = config["ixx"].as<double>();
    double iyy = config["iyy"].as<double>();
    double izz = config["izz"].as<double>();
    J_ = Eigen::Vector3d(ixx, iyy, izz).asDiagonal();

    prop_radius_ = 0.062;
    arm_length_ = 0.26;
    min_rpm_ = 1200;
    max_rpm_ = 35000;
    esc_factor_ = 1;

    state_.p = Eigen::Vector3d::Zero();
    state_.v = Eigen::Vector3d::Zero();
    state_.rot = gtsam::Rot3::Identity();
    state_.body_rate = Eigen::Vector3d::Zero();
    state_.motor_rpm = Eigen::Array4d::Zero();

    input_ = Eigen::Array4d::Zero();

    external_force_.setZero();
    external_torque_.setZero();

    // YAML::Node config   = YAML::LoadFile("../config/quadrotor_thrust_gyro.yaml");
    thrust_noise_mean = config["thrust_noise_mean"].as<double>();
    thrust_noise_cov = config["thrust_noise_cov"].as<double>();
    angular_speed_mean = config["angular_speed_mean"].as<double>();
    angular_speed_cov = config["angular_speed_cov"].as<double>();
    double drag_force_x = config["drag_force_x"].as<double>();
    double drag_force_y = config["drag_force_y"].as<double>();
    double drag_force_z = config["drag_force_z"].as<double>();
    float trj_len_max = config["trj_len_max"].as<double>();
    obs_num_ = config["obs_num"] ? config["obs_num"].as<uint16_t>() : 0;
    cylinder_num_ = config["cylinder_num"] ? config["cylinder_num"].as<uint16_t>() : 0;
    sphere_radius_ = config["obs1_radius"] ? config["obs1_radius"].as<double>() : 0.1;
    quad_size_ = config["uav_size"] ? config["uav_size"].as<double>() : 0.2;
    static_obs_num_ = config["static_obs_num"] ? config["static_obs_num"].as<uint16_t>() : 0;

    // double safe_d       = config["safe_d"].as<double>();
    drag_force_params_ = Eigen::Vector3d(drag_force_x, drag_force_y, drag_force_z);
    obstacles_.resize(obs_num_ + cylinder_num_);

    ui_ptr = std::make_shared<UI>(trj_len_max, obs_num_, quad_size_ / 2.0,
                                  trajectoryRecordPath("../config/quadrotor_thrust_gyro.yaml"));
    static_obstacles_.clear(); // Clear previous obstacles

    const float circleRadius = 1.5f;
    const float maxRadius = 0.20f;
    const float minRadius = 0.03f;
    const float centerZ = 1.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> angleDist(M_PI / 2, 3 * M_PI / 2); // Left half-circle
    std::uniform_real_distribution<double> radiusDist(minRadius, maxRadius);

    for (int i = 0; i < static_obs_num_; ++i) {
        Obstacle obstacle;
        bool collision;
        int attempts = 0;
        const int maxAttempts = 100; // Prevent infinite loops

        do {
            collision = false;
            double theta = angleDist(gen);
            double r = radiusDist(gen);
            double x = circleRadius * std::cos(theta);
            double y = circleRadius * std::sin(theta);

            obstacle.obs_pos = gtsam::Vector3(x, y, centerZ);
            obstacle.obs_vel = gtsam::Vector3::Zero();
            obstacle.obs_size = r;

            // Check collision with existing obstacles
            for (const auto& existing : static_obstacles_) {
                double dx = existing.obs_pos.x() - obstacle.obs_pos.x();
                double dy = existing.obs_pos.y() - obstacle.obs_pos.y();
                double dz = existing.obs_pos.z() - obstacle.obs_pos.z();
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (distance < (existing.obs_size + obstacle.obs_size)) {
                    collision = true;
                    break;
                }
            }

            if (++attempts >= maxAttempts) {
                IPN_LOG_WARNING << "Warning: Max attempts reached. Skipping obstacle.";
                break;
            }
        } while (collision);

        if (!collision) {
            static_obstacles_.push_back(obstacle);
        }
    }
    loadObstacles(config);
}

Quadrotor::Quadrotor(const std::string& yaml_file, bool enable_visualization) {
    // Existing constructor code
    YAML::Node config = YAML::LoadFile(yaml_file);
    g_ = config["g"].as<double>();
    mass_ = config["mass"].as<double>();
    kf_ = config["k_f"].as<double>();
    km_ = config["k_m"].as<double>();
    motor_time_constant_ = config["time_constant"].as<double>();

    double ixx = config["ixx"].as<double>();
    double iyy = config["iyy"].as<double>();
    double izz = config["izz"].as<double>();
    J_ = Eigen::Vector3d(ixx, iyy, izz).asDiagonal();

    prop_radius_ = 0.062;
    arm_length_ = 0.26;
    min_rpm_ = 1200;
    max_rpm_ = 35000;
    esc_factor_ = 1;

    state_.p = Eigen::Vector3d::Zero();
    state_.v = Eigen::Vector3d::Zero();
    state_.rot = gtsam::Rot3::Identity();
    state_.body_rate = Eigen::Vector3d::Zero();
    state_.motor_rpm = Eigen::Array4d::Zero();

    input_ = Eigen::Array4d::Zero();

    external_force_.setZero();
    external_torque_.setZero();

    // YAML::Node config   = YAML::LoadFile("../config/quadrotor_thrust_gyro.yaml");
    thrust_noise_mean = config["thrust_noise_mean"].as<double>();
    thrust_noise_cov = config["thrust_noise_cov"].as<double>();
    angular_speed_mean = config["angular_speed_mean"].as<double>();
    angular_speed_cov = config["angular_speed_cov"].as<double>();
    double drag_force_x = config["drag_force_x"].as<double>();
    double drag_force_y = config["drag_force_y"].as<double>();
    double drag_force_z = config["drag_force_z"].as<double>();
    float trj_len_max = config["trj_len_max"].as<double>();
    obs_num_ = config["obs_num"] ? config["obs_num"].as<uint16_t>() : 0;
    cylinder_num_ = config["cylinder_num"] ? config["cylinder_num"].as<uint16_t>() : 0;
    sphere_radius_ = config["obs1_radius"] ? config["obs1_radius"].as<double>() : 0.1;
    quad_size_ = config["uav_size"] ? config["uav_size"].as<double>() : 0.2;
    static_obs_num_ = config["static_obs_num"] ? config["static_obs_num"].as<uint16_t>() : 0;
    // double safe_d       = config["safe_d"].as<double>();
    drag_force_params_ = Eigen::Vector3d(drag_force_x, drag_force_y, drag_force_z);
    obstacles_.resize(obs_num_ + cylinder_num_);

    if (enable_visualization)
        ui_ptr = std::make_shared<UI>(trj_len_max, obs_num_, quad_size_ / 2.0,
                                      trajectoryRecordPath(yaml_file));
    static_obstacles_.clear(); // Clear previous obstacles

    const float circleRadius = 1.5f;
    const float maxRadius = 0.20f;
    const float minRadius = 0.03f;
    const float centerZ = 1.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> angleDist(M_PI / 2, 3 * M_PI / 2); // Left half-circle
    std::uniform_real_distribution<double> radiusDist(minRadius, maxRadius);

    for (int i = 0; i < static_obs_num_; ++i) {
        Obstacle obstacle;
        bool collision;
        int attempts = 0;
        const int maxAttempts = 100; // Prevent infinite loops

        do {
            collision = false;
            double theta = angleDist(gen);
            double r = radiusDist(gen);
            double x = circleRadius * std::cos(theta);
            double y = circleRadius * std::sin(theta);

            obstacle.obs_pos = gtsam::Vector3(x, y, centerZ);
            obstacle.obs_vel = gtsam::Vector3::Zero();
            obstacle.obs_size = r;

            // Check collision with existing obstacles
            for (const auto& existing : static_obstacles_) {
                double dx = existing.obs_pos.x() - obstacle.obs_pos.x();
                double dy = existing.obs_pos.y() - obstacle.obs_pos.y();
                double dz = existing.obs_pos.z() - obstacle.obs_pos.z();
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (distance < (existing.obs_size + obstacle.obs_size)) {
                    collision = true;
                    break;
                }
            }

            if (++attempts >= maxAttempts) {
                IPN_LOG_WARNING << "Warning: Max attempts reached. Skipping obstacle.";
                break;
            }
        } while (collision);

        if (!collision) {
            static_obstacles_.push_back(obstacle);
        }
    }
    loadObstacles(config);
}

namespace {
gtsam::Vector3 readVector3(const YAML::Node& node, const gtsam::Vector3& fallback) {
    if (!node || !node.IsSequence() || node.size() != 3) return fallback;
    return gtsam::Vector3(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}
} // namespace

void Quadrotor::loadObstacles(const YAML::Node& config) {
    const YAML::Node definitions = config["obstacles"];
    if (!definitions || !definitions.IsSequence() || definitions.size() == 0) return;

    obstacles_.clear();
    for (std::size_t i = 0; i < definitions.size(); ++i) {
        const YAML::Node node = definitions[i];
        if (node["enabled"] && !node["enabled"].as<bool>()) continue;
        Obstacle obstacle;
        obstacle.name = node["name"] ? node["name"].as<std::string>() : "obstacle_" + std::to_string(i);
        const std::string shape = node["shape"] ? node["shape"].as<std::string>() : "sphere";
        obstacle.obs_type = shape == "cylinder" ? ObsType::cylinder : shape == "box" ? ObsType::box : ObsType::sphere;
        obstacle.obs_pos = readVector3(node["position"], gtsam::Vector3::Zero());
        obstacle.initial_pos = obstacle.obs_pos;
        obstacle.obs_size = node["radius"] ? node["radius"].as<float>() : sphere_radius_;
        obstacle.obs_height = node["height"] ? node["height"].as<float>() : 1.0F;
        obstacle.half_extents = readVector3(node["half_extents"], gtsam::Vector3::Constant(obstacle.obs_size));

        const YAML::Node motion = node["motion"];
        const std::string style = motion && motion["style"] ? motion["style"].as<std::string>() : "stationary";
        obstacle.motion_type = style == "linear" ? MotionType::Linear : style == "ellipse" ? MotionType::Ellipse :
                               style == "ping_pong" ? MotionType::PingPong : MotionType::Stationary;
        obstacle.obs_vel = readVector3(motion ? motion["velocity"] : YAML::Node(), gtsam::Vector3::Zero());
        obstacle.motion_axis = readVector3(motion ? motion["axis"] : YAML::Node(), gtsam::Vector3::UnitX());
        if (obstacle.motion_axis.norm() > 1e-9) obstacle.motion_axis.normalize();
        obstacle.motion_amplitude = readVector3(motion ? motion["amplitude"] : YAML::Node(), gtsam::Vector3::Zero());
        obstacle.motion_speed = motion && motion["speed"] ? motion["speed"].as<double>() : obstacle.obs_vel.norm();
        obstacle.motion_phase = motion && motion["phase"] ? motion["phase"].as<double>() : 0.0;
        obstacles_.push_back(obstacle);
    }
    obs_num_ = static_cast<uint16_t>(obstacles_.size());
    cylinder_num_ = 0;
    IPN_LOG_INFO << "Loaded " << obstacles_.size() << " configured obstacles";
}

void Quadrotor::step(double dt) {
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
    if (vnorm.norm() != 0) {
        vnorm.normalize();
    }

    Eigen::Vector3d drag_force = -state_.rot.matrix() *
                                 Eigen::Matrix3d(drag_force_params_.asDiagonal()) *
                                 state_.rot.matrix().transpose() * state_.v;
    Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) +
                            state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                            external_force_ / mass_ + drag_force;

    Eigen::Vector3d p_dot = state_.v;

    // J* body_rate_dot = torque - J.cross(J* body_rate)
    Eigen::Vector3d body_rate_dot =
        J_.inverse() * (torque - state_.body_rate.cross(J_ * state_.body_rate) + external_torque_);

    // Predict state
    predicted_state_.p = state_.p + p_dot * dt;
    predicted_state_.v = state_.v + v_dot * dt;

    // predicted_state_.motor_rpm = state_.motor_rpm + (motor_rpm - state_.motor_rpm) /
    // motor_time_constant_;
    predicted_state_.rot = state_.rot * gtsam::Rot3::Expmap(state_.body_rate * dt);
    predicted_state_.body_rate = state_.body_rate + body_rate_dot * dt;

    state_ = predicted_state_;
    // Don't go below zero, simulate floor
    if (state_.p(2) < 0.0 && state_.v(2) < 0) {
        state_.p(2) = 0;
        state_.v(2) = 0;
    }
}

void Quadrotor::operator()(const Quadrotor::stateType& x, Quadrotor::stateType& dxdt,
                           const double /*t*/) {
    State est_state;
    Eigen::Matrix3d cur_rotm;
    for (int i = 0; i < 3; i++) {
        est_state.p(i) = x[0 + i];
        est_state.v(i) = x[3 + i];
        cur_rotm(i, 0) = x[6 + i];
        cur_rotm(i, 1) = x[9 + i];
        cur_rotm(i, 2) = x[12 + i];
        est_state.body_rate(i) = x[15 + i];
    }

    // Re-orthonormalize R (polar decomposition)
    Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
    Eigen::Matrix3d P = llt.matrixL();
    Eigen::Matrix3d R = cur_rotm * P.inverse();

    est_state.rot = gtsam::Rot3(R);

    Eigen::Vector3d vnorm;

    std::normal_distribution<double> thrust_noise(thrust_noise_mean, thrust_noise_cov);
    double at_noise = thrust_noise(generator_);

    double thrust = x[18] + at_noise; // cur force

    Eigen::Vector3d torque(x[19], x[20], x[21]); // cur torque

    vnorm = est_state.v;
    if (vnorm.norm() != 0) {
        vnorm.normalize();
    }

    Eigen::Vector3d drag_force = est_state.rot.matrix() *
                                 Eigen::Matrix3d(drag_force_params_.asDiagonal()) *
                                 est_state.rot.matrix().transpose() * est_state.v;

    Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) +
                            est_state.rot.rotate(gtsam::Vector3(0, 0, thrust / mass_)) +
                            external_force_ / mass_ + drag_force / mass_;

    Eigen::Vector3d p_dot = est_state.v;

    Eigen::Matrix3d r_dot = est_state.rot.matrix() * gtsam::skewSymmetric(est_state.body_rate);

    // J* body_rate_dot = torque - J.cross(J* body_rate)
    Eigen::Vector3d body_rate_dot =
        J_.inverse() *
        (torque - est_state.body_rate.cross(J_ * est_state.body_rate) + external_torque_);

    for (int i = 0; i < 3; i++) {
        dxdt[0 + i] = p_dot(i);
        dxdt[3 + i] = v_dot(i);
        dxdt[6 + i] = r_dot(i, 0);
        dxdt[9 + i] = r_dot(i, 1);
        dxdt[12 + i] = r_dot(i, 2);
        dxdt[15 + i] = body_rate_dot(i);
    }

    for (int i = 0; i < 4; i++) {
        dxdt[18 + i] = (thrust_torque_[i] - state_.thrust_torque[i]) / motor_time_constant_;
    }

    for (int i = 0; i < 22; ++i) {
        if (std::isnan(dxdt[i])) {
            dxdt[i] = 0;
            IPN_LOG_DEBUG << "Non-finite derivative replaced with zero: state_index=" << i;
        }
    }
}

void Quadrotor::stepODE(double dt, gtsam::Vector4 fm) {
    stateType x;
    Eigen::Matrix3d r = state_.rot.matrix();
    for (int i = 0; i < 3; i++) {
        x[0 + i] = state_.p(i);
        x[3 + i] = state_.v(i);
        x[6 + i] = r(i, 0);
        x[9 + i] = r(i, 1);
        x[12 + i] = r(i, 2);
        x[15 + i] = state_.body_rate(i);
    }

    // float sin_force = 0.1* sin()
    // state_.thrust_torque[0] = state_.thrust_torque[0];

    for (int i = 0; i < 4; i++) {

        x[18 + i] = state_.thrust_torque[i];
    }

    thrust_torque_ = fm; // control at future dt.
    integrate(boost::ref(*this), x, 0.0, dt, dt);

    Eigen::Matrix3d cur_rotm;
    for (int i = 0; i < 3; i++) {
        state_.p(i) = x[0 + i];
        state_.v(i) = x[3 + i];
        cur_rotm(i, 0) = x[6 + i];
        cur_rotm(i, 1) = x[9 + i];
        cur_rotm(i, 2) = x[12 + i];
        state_.body_rate(i) = x[15 + i];
    }

    for (int i = 0; i < 4; i++) {
        state_.thrust_torque[i] = x[18 + i];
    }

    // Re-orthonormalize R (polar decomposition)
    Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
    Eigen::Matrix3d P = llt.matrixL();
    Eigen::Matrix3d R = cur_rotm * P.inverse();

    state_.rot = gtsam::Rot3(R);

    std::normal_distribution<double> angular_speed_noise(angular_speed_mean, angular_speed_cov);
    gtsam::Vector3 ome_noise =
        gtsam::Vector3(angular_speed_noise(generator_), angular_speed_noise(generator_),
                       0.01 * angular_speed_noise(generator_));

    state_.p = state_.p;
    state_.v = state_.v;
    state_.body_rate = state_.body_rate + ome_noise;

    // printCurState();
}

Eigen::Matrix4d Quadrotor::ComputeEffectivenessMatrix() {
    Eigen::Matrix4d effectivenessMatrix;

    // effectivenessMatrix <<
    //     kf_, kf_, kf_, kf_,
    //     0, 0, arm_length_ * kf_, - arm_length_ * kf_,
    //     - arm_length_ * kf_, arm_length_ * kf_, 0, 0,
    //     km_, km_, -km_, -km_;
    double dx0 = 0.10f;
    double dx1 = 0.10f;

    effectivenessMatrix << kf_, kf_, kf_, kf_, dx0 * kf_, -dx0 * kf_, -dx0 * kf_, dx0 * kf_,
        -dx1 * kf_, -dx1 * kf_, dx0 * kf_, dx0 * kf_, km_ * kf_, -km_ * kf_, km_ * kf_, -km_ * kf_;

    IPN_LOG_DEBUG << "Rotor effectiveness matrix normalized by thrust coefficient: "
                  << effectivenessMatrix / kf_;
    return effectivenessMatrix;
}

Eigen::Vector4d Quadrotor::computeRotorVelocities() {
    effectiveness_ = ComputeEffectivenessMatrix();

    Eigen::Vector4d thrust;
    thrust = effectiveness_.inverse() * thrust_torque_;
    Eigen::Vector4d identity = Eigen::Vector4d::Identity();

    // esc_factor* Actuator_Ouput^2 + (1 - esc_factor)* Actuator_Output - rotor_thrust = 0

    Eigen::Vector4d actuator_output;
    double a = esc_factor_;
    double b = 1 - esc_factor_;

    actuator_output =
        (-b * identity + (Eigen::Vector4d)(b * b * identity - 4 * a * (-thrust)).array().sqrt()) /
        (2 * a);
    input_ = actuator_output;

    return actuator_output;
}

Eigen::Vector4d Quadrotor::inverseRotorVelocities(Eigen::Vector4d rotor_speed) {
    effectiveness_ = ComputeEffectivenessMatrix();

    Eigen::Vector4d actuator_output;

    Eigen::Vector4d thrust_torque;
    actuator_output = esc_factor_ * rotor_speed.cwiseAbs2() + (1 - esc_factor_) * rotor_speed;

    thrust_torque = effectiveness_ * actuator_output;
    return thrust_torque;

    // esc_factor* Actuator_Ouput^2 + (1 - esc_factor)* Actuator_Output - rotor_thrust = 0
}

void Quadrotor::printCurState() {
    IPN_LOG_DEBUG << "State position [m]: " << state_.p.transpose();
    IPN_LOG_DEBUG << "State rotation logmap [rad]: " << Rot3::Logmap(state_.rot).transpose();
    IPN_LOG_DEBUG << "State velocity [m/s]: " << state_.v.transpose();
    IPN_LOG_DEBUG << "State body rate [rad/s]: " << state_.body_rate.transpose();
    IPN_LOG_DEBUG << "State thrust and torque: " << state_.thrust_torque.transpose();
}
void Quadrotor::setInput(gtsam::Vector4 thrust_torque) {
    thrust_torque_ = thrust_torque;
}

void Quadrotor::setInput(double u1, double u2, double u3, double u4) {
    input_(0) = u1;
    input_(1) = u2;
    input_(2) = u3;
    input_(3) = u4;
    for (int i = 0; i < 4; i++) {
        if (std::isnan(input_(i))) {
            input_(i) = (max_rpm_ + min_rpm_) / 2;
            IPN_LOG_DEBUG << "Non-finite control input detected";
        }
        if (input_(i) > max_rpm_)
            input_(i) = max_rpm_;
        else if (input_(i) < min_rpm_)
            input_(i) = min_rpm_;
    }
    state_.motor_rpm << u1, u2, u3, u4;
}

const State& Quadrotor::getState(void) const {
    return state_;
}
void Quadrotor::setState(const State& state) {
    state_ = state;
    // state_.p             = state.p;
    // state_.v             = state.v;
    // state_.rot           = state.rot;
    // state_.body_rate     = state.body_rate;
    // state_.motor_rpm     = state.motor_rpm;
    // state_.thrust_torque = state.thrust_torque;
    // state_.timestamp     = state.timestamp;
}

void Quadrotor::setStatePos(const Eigen::Vector3d& Pos) {
    state_.p = Pos;
}

double Quadrotor::getMass(void) const {
    return mass_;
}
void Quadrotor::setMass(double mass) {
    mass_ = mass;
}

double Quadrotor::getGravity(void) const {
    return g_;
}
void Quadrotor::setGravity(double g) {
    g_ = g;
}

const Eigen::Matrix3d& Quadrotor::getInertia(void) const {
    return J_;
}
void Quadrotor::setInertia(const Eigen::Matrix3d& inertia) {
    if (inertia != inertia.transpose()) {
        IPN_LOG_WARNING << "Inertia matrix not symmetric, not setting";
        return;
    }
    J_ = inertia;
}

double Quadrotor::getArmLength(void) const {
    return arm_length_;
}
void Quadrotor::setArmLength(double d) {
    if (d <= 0) {
        IPN_LOG_WARNING << "Arm length <= 0, not setting";
        return;
    }

    arm_length_ = d;
}

double Quadrotor::getPropRadius(void) const {
    return prop_radius_;
}
void Quadrotor::setPropRadius(double r) {
    if (r <= 0) {
        IPN_LOG_WARNING << "Prop radius <= 0, not setting";
        return;
    }
    prop_radius_ = r;
}

double Quadrotor::getPropellerThrustCoefficient(void) const {
    return kf_;
}
void Quadrotor::setPropellerThrustCoefficient(double kf) {
    if (kf <= 0) {
        IPN_LOG_WARNING << "Thrust coefficient <= 0, not setting";
        return;
    }

    kf_ = kf;
}

double Quadrotor::getPropellerMomentCoefficient(void) const {
    return km_;
}
void Quadrotor::setPropellerMomentCoefficient(double km) {
    if (km <= 0) {
        IPN_LOG_WARNING << "Moment coefficient <= 0, not setting";
        return;
    }

    km_ = km;
}

double Quadrotor::getMotorTimeConstant(void) const {
    return motor_time_constant_;
}
void Quadrotor::setMotorTimeConstant(double k) {
    if (k <= 0) {
        IPN_LOG_WARNING << "Motor time constant <= 0, not setting";
        return;
    }

    motor_time_constant_ = k;
}

const Eigen::Vector3d& Quadrotor::getExternalForce(void) const {
    return external_force_;
}
void Quadrotor::setExternalForce(const Eigen::Vector3d& force) {
    external_force_ = force;
}

const Eigen::Vector3d& Quadrotor::getExternalMoment(void) const {
    return external_torque_;
}
void Quadrotor::setExternalMoment(const Eigen::Vector3d& torque) {
    external_torque_ = torque;
}

double Quadrotor::getMaxRPM(void) const {
    return max_rpm_;
}
void Quadrotor::setMaxRPM(double max_rpm) {
    if (max_rpm <= 0) {
        IPN_LOG_WARNING << "Max rpm <= 0, not setting";
        return;
    }
    max_rpm_ = max_rpm;
}

double Quadrotor::getMinRPM(void) const {
    return min_rpm_;
}
void Quadrotor::setMinRPM(double min_rpm) {
    if (min_rpm < 0) {
        IPN_LOG_WARNING << "Min rpm < 0, not setting";
        return;
    }
    min_rpm_ = min_rpm;
}

Eigen::Vector3d Quadrotor::getAcc() const {
    return acc_;
}

bool Quadrotor::renderHistoryOpt(std::vector<State>& pred_trj, boost::optional<gtsam::Vector3&> err,
                                 boost::optional<Features&> features,
                                 boost::optional<gtsam::Vector3&> vicon_measurement,
                                 boost::optional<gtsam::Vector3&> rot_err,
                                 boost::optional<std::vector<State>&> ref_trj,
                                 boost::optional<float&> opt_cost) {
    clock_ = clock_ + 0.01f;
    return ui_ptr ? ui_ptr->renderHistoryOpt(state_, pred_trj, err, features, vicon_measurement,
                                              rot_err, ref_trj, opt_cost, obstacles_)
                  : true;
    // unsigned int microsecond = 1000000;
    // usleep(0.1 * microsecond);
}

gtsam::Vector3 Quadrotor::getObsbyEllipse(uint8_t index) {
    gtsam::Vector3 point3d{0, 0, 0};
    if (index >= obs_num_) {
        return point3d;
    }
    float a = 1.10;
    float b = 0.50;
    float v = 0.40;
    float z = 1.00;

    // clock_ = 10.0; // static obstacles

    double t = clock_ + index * 2.0 * M_PI / obs_num_ * sqrt(a * a + b * b) /
                            v; // Spread obstacles evenly over one cycle

    double angle = v * t / sqrt(a * a + b * b);
    double x = a * cos(angle);
    double y = b * sin(angle);

    // // rotating
    // double theta = M_PI / 4;
    // double rotatedX = x * cos(theta) - y * sin(theta);
    // double rotatedY = x * sin(theta) + y * cos(theta);

    point3d[0] = x - a / 2;
    point3d[1] = y - b / 2;
    point3d[2] = z;
    return point3d;
}

bool checkCollision(const Obstacle& a, const Obstacle& b) {
    double dx = a.obs_pos.x() - b.obs_pos.x();
    double dy = a.obs_pos.y() - b.obs_pos.y();
    double distance_sq = dx * dx + dy * dy;
    double min_distance = a.obs_size + b.obs_size;
    return distance_sq < (min_distance * min_distance);
}

void Quadrotor::initializeObstacles() {
    const float circleRadius = 1.5f;
    const float maxRadius = 0.20f;
    const float minRadius = 0.10f;
    const float centerZ = 1.0f;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> angleDist(M_PI / 2, 3 * M_PI / 2);
    std::uniform_real_distribution<double> radiusDist(minRadius, maxRadius);
    std::uniform_real_distribution<double> velDist(-0.1, 0.1);

    for (int i = 0; i < static_obs_num_; ++i) {
        Obstacle obstacle;
        bool collision;
        int attempts = 0;
        const int maxAttempts = 100;

        do {
            collision = false;
            double theta = angleDist(gen);
            double r = radiusDist(gen);
            double x = circleRadius * std::cos(theta);
            double y = circleRadius * std::sin(theta);

            obstacle.obs_pos = gtsam::Vector3(x, y, centerZ);
            obstacle.obs_vel = gtsam::Vector3(velDist(gen), velDist(gen), 0.0);
            obstacle.obs_size = r;

            for (const auto& existing : static_obstacles_) {
                if (checkCollision(obstacle, existing)) {
                    collision = true;
                    break;
                }
            }

            if (++attempts >= maxAttempts) {
                IPN_LOG_WARNING << "Warning: Max attempts reached. Skipping obstacle.";
                break;
            }
        } while (collision);

        if (!collision) {
            static_obstacles_.push_back(obstacle);
        }
    }
}

void Quadrotor::resolveCollisions() {
    const double response_factor = 0.5; // How much to push apart

    for (size_t i = 0; i < static_obstacles_.size(); ++i) {
        for (size_t j = i + 1; j < static_obstacles_.size(); ++j) {
            Obstacle& a = static_obstacles_[i];
            Obstacle& b = static_obstacles_[j];

            if (checkCollision(a, b)) {
                // Calculate collision normal and overlap
                gtsam::Vector3 delta = a.obs_pos - b.obs_pos;
                double distance = delta.norm();
                double overlap = (a.obs_size + b.obs_size) - distance;

                if (distance > 0) {
                    gtsam::Vector3 collision_normal = delta / distance;

                    // Push apart
                    double push = response_factor * overlap;
                    a.obs_pos += collision_normal * push;
                    b.obs_pos -= collision_normal * push;

                    // Adjust velocities to avoid sticking
                    double dot_product =
                        a.obs_vel.dot(collision_normal) - b.obs_vel.dot(collision_normal);
                    if (dot_product < 0) { // Moving toward each other
                        gtsam::Vector3 impulse = collision_normal * dot_product;
                        a.obs_vel -= impulse * 0.5;
                        b.obs_vel += impulse * 0.5;
                    }
                }
            }
        }
    }
}

template <typename T> const T& clamp(const T& value, const T& min, const T& max) {
    return (value < min) ? min : (max < value) ? max : value;
}

void Quadrotor::updateObstaclePositions(double dt) {
    if (!obstacles_.empty() && !obstacles_.front().name.empty()) {
        for (auto& obstacle : obstacles_) {
            const double phase = obstacle.motion_speed * dt + obstacle.motion_phase;
            const gtsam::Vector3 configured_velocity = obstacle.obs_vel;
            obstacle.obs_pos = obstacle.initial_pos;
            obstacle.obs_vel.setZero();
            if (obstacle.motion_type == MotionType::Linear) {
                obstacle.obs_vel = configured_velocity;
                obstacle.obs_pos += configured_velocity * dt;
            } else if (obstacle.motion_type == MotionType::Ellipse) {
                obstacle.obs_pos += gtsam::Vector3(obstacle.motion_amplitude.x() * std::cos(phase),
                                                   obstacle.motion_amplitude.y() * std::sin(phase),
                                                   obstacle.motion_amplitude.z() * std::sin(phase));
                obstacle.obs_vel = obstacle.motion_speed * gtsam::Vector3(-obstacle.motion_amplitude.x() * std::sin(phase),
                                                                          obstacle.motion_amplitude.y() * std::cos(phase),
                                                                          obstacle.motion_amplitude.z() * std::cos(phase));
            } else if (obstacle.motion_type == MotionType::PingPong) {
                obstacle.obs_pos += obstacle.motion_axis * obstacle.motion_amplitude.norm() * std::sin(phase);
                obstacle.obs_vel = obstacle.motion_axis * obstacle.motion_amplitude.norm() * obstacle.motion_speed * std::cos(phase);
            }
        }
        return;
    }
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
            if (obstacle.obs_vel(i) > 0.1)
                obstacle.obs_vel(i) = 0.1;
            if (obstacle.obs_vel(i) < -0.1)
                obstacle.obs_vel(i) = -0.1;
        }

        // Boundary checking
        const double maxX = 2.0, minX = -2.0;
        const double maxY = 2.0, minY = -2.0;

        if (obstacle.obs_pos.x() > maxX || obstacle.obs_pos.x() < minX) {
            obstacle.obs_vel.x() *= -1;
            obstacle.obs_pos.x() = clamp(obstacle.obs_pos.x(), minX, maxX);
        }
        if (obstacle.obs_pos.y() > maxY || obstacle.obs_pos.y() < minY) {
            obstacle.obs_vel.y() *= -1;
            obstacle.obs_pos.y() = clamp(obstacle.obs_pos.y(), minY, maxY);
        }
    }
}

Obstacle Quadrotor::getObsbyEllipsev(uint8_t index) {
    Obstacle obstacle{};
    if (index >= obs_num_) {
        return obstacle; // returns default obstacle (zero position and velocity)
    }

    if (index > static_obs_num_ - 1) {
        // Parameters
        float a = 1.10; // semi-major axis
        float b = 0.50; // semi-minor axis
        float v = 2.40; // velocity parameter
        float z = 1.00; // fixed height

        // Calculate time parameter with even spacing
        double t = clock_ + (index - static_obs_num_ + 1) * 2.0 * M_PI /
                                (obs_num_ - static_obs_num_) * sqrt(a * a + b * b) / v;

        // Position calculation
        double angle = v * t / sqrt(a * a + b * b);
        double x = a * cos(angle);
        double y = b * sin(angle);

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
        obstacle.obs_vel[2] = 0; // no vertical movement
        obstacle.obs_type = ObsType::cylinder;
        obstacle.obs_size = sphere_radius_;
        return obstacle;
    } else {
        // updateObstaclePositions(dt_);
        return static_obstacles_[index];
    }
}
} // namespace QuadrotorSim_SO3
