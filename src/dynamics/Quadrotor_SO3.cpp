#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <boost/bind.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace QuadrotorSim_SO3
{

    Quadrotor::Quadrotor(void)
    {
        g_ = 9.81;
        mass_ = 0.98; // 0.5;
        double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
        prop_radius_ = 0.062;

        /*Force moment*/
        J_ = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

        kf_ = 8.98132e-9; //  xy-moment k-gain
        // km_ = 2.5e-9; // from Nate
        // km = (Cq/Ct)*Dia*kf
        // Cq/Ct for 8 inch props from UIUC prop db ~ 0.07
        km_ = 0.07 * (3 * prop_radius_) * kf_; // z-moment k-gain

        arm_length_          = 0.26;
        motor_time_constant_ = 1.0 / 30;
        min_rpm_             = 1200;
        max_rpm_             = 35000;
        esc_factor_          = 0.7;

        state_.x         = Eigen::Vector3d::Zero();     // position
        state_.v         = Eigen::Vector3d::Zero();     // velocity
        state_.rot       = gtsam::Rot3::identity();   // altitude
        state_.omega     = Eigen::Vector3d::Zero(); // angular velocity
        state_.motor_rpm = Eigen::Array4d::Zero();

        last_state_      = state_;
        input_           = Eigen::Array4d::Zero();

        external_force_.setZero();

        displaySetup();

        record_info_.open("../data/record_info.txt");

        YAML::Node config = YAML::LoadFile("../config/quadrotor.yaml");
        AT_NOISE_MEAN       = config["AT_NOISE_MEAN"].as<double>();
        AT_NOISE_COV        = config["AT_NOISE_COV"].as<double>();
        ANGULAR_SPEED_MEAN  = config["ANGULAR_SPEED_MEAN"].as<double>();
        ANGULAR_SPEED_COV   = config["ANGULAR_SPEED_COV"].as<double>();
        double DRAG_FORCE_X = config["DRAG_FORCE_X"].as<double>();
        double DRAG_FORCE_Y = config["DRAG_FORCE_Y"].as<double>();
        double DRAG_FORCE_Z = config["DRAG_FORCE_Z"].as<double>();
        drag_force_p_ = Eigen::Vector3d(DRAG_FORCE_X, DRAG_FORCE_Y, DRAG_FORCE_Z);
    }

    void Quadrotor::step(double dt)
    {
        State predicted_state_;

        Eigen::Vector3d vnorm;
        Eigen::Array4d motor_rpm_sq;

        motor_rpm_sq = state_.motor_rpm.array().square();

        double thrust = kf_ * motor_rpm_sq.sum();

        Eigen::Vector3d moments;
        moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
        moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
        moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) - motor_rpm_sq(3));

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -state_.rot.matrix() * Eigen::Matrix3d(drag_force_p_.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot = - Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_ + drag_force;

        Eigen::Vector3d p_dot = state_.v;

        // J* omega_dot = moments - J.cross(J* omega)
        Eigen::Vector3d omega_dot = J_.inverse() * (moments - state_.omega.cross(J_ * state_.omega) + external_moment_);

        // noise
        // std::default_random_engine generator;
        // std::normal_distribution<double> x_noise(0.0, 0.05);
        // std::normal_distribution<double> r_noise(0.0, 0.005);
        // std::normal_distribution<double> v_noise(0.0, 0.005);
        // std::normal_distribution<double> o_noise(0.0, 0.005);

        // Predict state
        double dt22 = 0.5 * dt * dt;
        predicted_state_.x = state_.x + p_dot * dt;                                                    // + dt* gtsam::Vector3(x_noise(generator), x_noise(generator), x_noise(generator));
        predicted_state_.v = state_.v + v_dot * dt;                                                    // + dt* gtsam::Vector3(v_noise(generator), v_noise(generator), v_noise(generator));
        predicted_state_.rot = state_.rot * gtsam::Rot3::Expmap(state_.omega * dt + omega_dot * dt22); // + dt* gtsam::Vector3(r_noise(generator), r_noise(generator), r_noise(generator)) );
        predicted_state_.omega = state_.omega + omega_dot * dt;                                        // + dt* gtsam::Vector3(o_noise(generator), o_noise(generator), o_noise(generator));
        predicted_state_.motor_rpm = state_.motor_rpm;

        state_ = predicted_state_;
        // Don't go below zero, simulate floor
        if (state_.x(2) < 0.0 && state_.v(2) < 0)
        {
            state_.x(2) = 0;
            state_.v(2) = 0;
        }
    }

    void Quadrotor::operator()(const Quadrotor::stateType &x, Quadrotor::stateType &dxdt, const double t)
    {
        State cur_state;
        Eigen::Matrix3d cur_rotm;
        for (int i = 0; i < 3; i++)
        {
            cur_state.x(i) = x[0 + i];
            cur_state.v(i) = x[3 + i];
            cur_rotm(i, 0) = x[6 + i];
            cur_rotm(i, 1) = x[9 + i];
            cur_rotm(i, 2) = x[12 + i];
            cur_state.omega(i) = x[15 + i];
        }

        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
        Eigen::Matrix3d P = llt.matrixL();
        Eigen::Matrix3d R = cur_rotm * P.inverse();

        cur_state.rot = gtsam::Rot3(R);

        Eigen::Vector3d vnorm;

        std::normal_distribution<double> aT_noise(AT_NOISE_MEAN, AT_NOISE_COV);
        double at_noise = aT_noise(generator_);

        double thrust = x[18] + at_noise; // cur force

        Eigen::Vector3d moment(x[19], x[20], x[21]); // cur moment

        vnorm = cur_state.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = - cur_state.rot.matrix() * Eigen::Matrix3d(drag_force_p_.asDiagonal()) * cur_state.rot.matrix().transpose() * cur_state.v;
        
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + cur_state.rot.rotate(gtsam::Vector3(0, 0, thrust / mass_)) +
                                external_force_ / mass_ + drag_force;
        // std::cout << "v_dot i stepODE opertor: " << v_dot << std::endl;

        Eigen::Vector3d p_dot  = cur_state.v;

        Eigen::Matrix3d r_dot = cur_state.rot.matrix() * gtsam::skewSymmetric(cur_state.omega);

        // J* omega_dot = moment - J.cross(J* omega)
        Eigen::Vector3d omega_dot = J_.inverse() * (moment - cur_state.omega.cross(J_ * cur_state.omega) + external_moment_);

        for (int i = 0; i < 3; i++)
        {
            dxdt[0 + i] = p_dot(i);
            dxdt[3 + i] = v_dot(i);
            dxdt[6 + i] = r_dot(i, 0);
            dxdt[9 + i] = r_dot(i, 1);
            dxdt[12 + i] = r_dot(i, 2);
            dxdt[15 + i] = omega_dot(i);
        }

        for (int i = 0; i < 4; i++)
        {
            dxdt[18 + i] = (force_moment_[i] - state_.force_moment[i]) / 0.01f;
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
            x[0 + i] = state_.x(i);
            x[3 + i] = state_.v(i);
            x[6 + i] = r(i, 0);
            x[9 + i] = r(i, 1);
            x[12 + i] = r(i, 2);
            x[15 + i] = state_.omega(i);
        }



        state_.force_moment[0] = state_.force_moment[0];

        for (int i = 0; i < 4; i++)
        {

            x[18 + i] = state_.force_moment[i];
        }

        force_moment_ = fm; // control at future dt.
        integrate(boost::ref(*this), x, 0.0, dt, dt);

        Eigen::Matrix3d cur_rotm;
        for (int i = 0; i < 3; i++)
        {
            state_.x(i) = x[0 + i];
            state_.v(i) = x[3 + i];
            cur_rotm(i, 0) = x[6 + i];
            cur_rotm(i, 1) = x[9 + i];
            cur_rotm(i, 2) = x[12 + i];
            state_.omega(i) = x[15 + i];
        }

        for (int i = 0; i < 4; i++)
        {
            state_.force_moment[i] = x[18 + i];
        }

        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
        Eigen::Matrix3d P = llt.matrixL();
        Eigen::Matrix3d R = cur_rotm * P.inverse();

        state_.rot = gtsam::Rot3(R);

        // std::normal_distribution<double> x_noise(0.0, 0.05);
        // std::normal_distribution<double> r_noise(0.0, 0.005);
        // std::normal_distribution<double> v_noise(0.0, 0.20);
        std::normal_distribution<double> o_noise(ANGULAR_SPEED_MEAN, ANGULAR_SPEED_COV);

        // gtsam::Vector3 pos_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(x_noise(generator_), x_noise(generator_), x_noise(generator_));
        // gtsam::Vector3 vel_noise = dt* gtsam::Vector3(v_noise(generator_), v_noise(generator_), v_noise(generator_));
        // gtsam::Vector3 rot_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(r_noise(generator_), r_noise(generator_), r_noise(generator_));
        gtsam::Vector3 ome_noise = gtsam::Vector3(o_noise(generator_), o_noise(generator_), o_noise(generator_));

        state_.x = state_.x;
        state_.v = state_.v;
        state_.omega = state_.omega + ome_noise;

        std::cout << "state_.omega: " << state_.omega.transpose() << " , ome_noise: " << ome_noise.transpose() << std::endl;
        printCurState();
    }

    void Quadrotor::stepODE2(double dt, gtsam::Vector4 fm)
    {
        stateType x;
        Eigen::Matrix3d r = state_.rot.matrix();
        for (int i = 0; i < 3; i++)
        {
            x[0 + i] = state_.x(i);
            x[3 + i] = state_.v(i);
            x[6 + i] = r(i, 0);
            x[9 + i] = r(i, 1);
            x[12 + i] = r(i, 2);
            x[15 + i] = state_.omega(i);
        }

        for (int i = 0; i < 4; i++)
        {
            x[18 + i] = state_.force_moment[i];
        }

        force_moment_ = fm; // control at future dt.
        integrate(boost::ref(*this), x, 0.0, dt, dt / 10);

        Eigen::Matrix3d cur_rotm;
        for (int i = 0; i < 3; i++)
        {
            state_.x(i) = x[0 + i];
            state_.v(i) = x[3 + i];
            cur_rotm(i, 0) = x[6 + i];
            cur_rotm(i, 1) = x[9 + i];
            cur_rotm(i, 2) = x[12 + i];
            state_.omega(i) = x[15 + i];
        }

        for (int i = 0; i < 4; i++)
        {
            state_.force_moment[i] = x[18 + i];
        }

        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
        Eigen::Matrix3d P = llt.matrixL();
        Eigen::Matrix3d R = cur_rotm * P.inverse();

        state_.rot = gtsam::Rot3(R);

        printCurState();
    }

    Eigen::Matrix4d Quadrotor::ComputeEffectivenessMatrix()
    {
        Eigen::Matrix4d effectivenessMatrix;

        effectivenessMatrix << 
            kf_, kf_, kf_, kf_,
            0, 0, arm_length_ * kf_, - arm_length_ * kf_,
            - arm_length_ * kf_, arm_length_ * kf_, 0, 0,
            km_, km_, -km_, -km_;

        return effectivenessMatrix;
    }

    Eigen::Vector4d Quadrotor::CumputeRotorsVel()
    {
        effectiveness_ = ComputeEffectivenessMatrix();

        Eigen::Vector4d thrust;
        thrust = effectiveness_.inverse()* force_moment_;
        Eigen::Vector4d identity = Eigen::Vector4d::Identity();

        // esc_factor* Actuator_Ouput^2 + (1 - esc_factor)* Actuator_Output - rotor_thrust = 0
        
        Eigen::Vector4d actuator_output;
        double a = esc_factor_;
        double b = 1 - esc_factor_;
    
        actuator_output = (- b * identity + (Eigen::Vector4d)(b * b * identity - 4 * a * (- thrust)).array().sqrt()) / (2 * a);
        input_ = actuator_output;
        
        return actuator_output;
    }

    void Quadrotor::printCurState()
    {
        Color::Modifier red(Color::FG_RED);
        Color::Modifier def(Color::FG_DEFAULT);
        Color::Modifier green(Color::FG_GREEN);
        std::cout << red;
        std::cout << "Cur Position: " << state_.x.transpose() << std::endl;
        std::cout << "Cur Rotation: " << Rot3::Logmap(state_.rot).transpose() << std::endl;
        std::cout << "Cur Velocity: " << state_.v.transpose() << std::endl;
        std::cout << "Cur    Omega: " << state_.omega.transpose() << std::endl;
        std::cout << "Cur ForceMom: " << state_.force_moment.transpose() << std::endl;
        std::cout << def;
    }
    void Quadrotor::setInput(gtsam::Vector4 force_moment)
    {
        force_moment_ = force_moment;
    }

    void Quadrotor::stepTMDeprecated(double dt, gtsam::Vector4 fm)
    {
        State predicted_state_;

        Eigen::Vector3d vnorm;

        double thrust = fm[0];

        Eigen::Vector3d moments = fm.tail(3);

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -state_.rot.matrix() * Eigen::Matrix3d(drag_force_p_.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_ + drag_force;

        Eigen::Vector3d p_dot = state_.v;
        // Eigen::Matrix3d r_dot = state_.rot.matrix() * gtsam::skewSymmetric(state_.omega);

        // J* omega_dot = moments - J.cross(J* omega)
        Eigen::Vector3d omega_dot = J_.inverse() * (moments - state_.omega.cross(J_ * state_.omega) + external_moment_);

        // noise
        std::normal_distribution<double> x_noise(0.0, 0.05);
        std::normal_distribution<double> r_noise(0.0, 0.005);
        std::normal_distribution<double> v_noise(0.0, 0.005);
        std::normal_distribution<double> o_noise(0.0, 0.005);

        gtsam::Vector3 pos_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(x_noise(generator_), x_noise(generator_), x_noise(generator_));
        gtsam::Vector3 vel_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(v_noise(generator_), v_noise(generator_), v_noise(generator_));
        gtsam::Vector3 rot_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(r_noise(generator_), r_noise(generator_), r_noise(generator_));
        gtsam::Vector3 ome_noise = dt * gtsam::Vector3(o_noise(generator_), o_noise(generator_), o_noise(generator_));

        predicted_state_.x = state_.x + p_dot * dt + pos_noise;
        predicted_state_.v = state_.v + v_dot * dt + vel_noise;
        predicted_state_.rot = state_.rot * gtsam::Rot3::Expmap(state_.omega * dt + rot_noise); // + omega_dot* dt22 +
        predicted_state_.omega = state_.omega + omega_dot * dt;                                 // + ome_noise;
        predicted_state_.motor_rpm = state_.motor_rpm;

        // predicted_state_.x = state_.x + p_dot * dt;
        // predicted_state_.v = state_.v + v_dot * dt;
        // predicted_state_.rot = state_.rot * gtsam::Rot3::Expmap(state_.omega * dt); // + omega_dot* dt22 +
        // predicted_state_.omega = state_.omega + omega_dot * dt;// + ome_noise;
        // predicted_state_.motor_rpm = state_.motor_rpm;

        state_ = predicted_state_;
        // Don't go below zero, simulate floor
        if (state_.x(2) < 0.0 && state_.v(2) < 0)
        {
            state_.x(2) = 0;
            state_.v(2) = 0;
        }
    }

    void Quadrotor::stepAddNoise(double dt)
    {
        State predicted_state_;

        Eigen::Vector3d vnorm;
        Eigen::Array4d motor_rpm_sq;

        motor_rpm_sq = state_.motor_rpm.array().square();

        double thrust = kf_ * motor_rpm_sq.sum();

        Eigen::Vector3d moments;
        moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
        moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
        moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) - motor_rpm_sq(3));

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -state_.rot.matrix() * Eigen::Matrix3d(drag_force_p_.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_ + drag_force;

        Eigen::Vector3d p_dot = state_.v;

        // J* omega_dot = moments - J.cross(J* omega)
        Eigen::Vector3d omega_dot = J_.inverse() * (moments - state_.omega.cross(J_ * state_.omega) + external_moment_);

        // noise
        std::normal_distribution<double> x_noise(0.0, 0.05);
        std::normal_distribution<double> r_noise(0.0, 0.005);
        std::normal_distribution<double> v_noise(0.0, 0.005);
        std::normal_distribution<double> o_noise(0.0, 0.005);

        // Predict state
        gtsam::Vector3 pos_noise = dt * gtsam::Vector3(x_noise(generator_), x_noise(generator_), x_noise(generator_));
        gtsam::Vector3 vel_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(v_noise(generator_), v_noise(generator_), v_noise(generator_));
        gtsam::Vector3 rot_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(r_noise(generator_), r_noise(generator_), r_noise(generator_));
        gtsam::Vector3 ome_noise = dt * gtsam::Vector3(o_noise(generator_), o_noise(generator_), o_noise(generator_));

        predicted_state_.x = state_.x + p_dot * dt + pos_noise;
        predicted_state_.v = state_.v + v_dot * dt + vel_noise;
        predicted_state_.rot = state_.rot * gtsam::Rot3::Expmap(state_.omega * dt + rot_noise); // + omega_dot* dt22 +
        predicted_state_.omega = state_.omega + omega_dot * dt;                                 // + ome_noise;
        predicted_state_.motor_rpm = state_.motor_rpm;

        state_ = predicted_state_;
        // Don't go below zero, simulate floor
        if (state_.x(2) < 0.0 && state_.v(2) < 0)
        {
            state_.x(2) = 0;
            state_.v(2) = 0;
        }
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

    const Quadrotor::State &Quadrotor::getState(void) const
    {
        return state_;
    }
    void Quadrotor::setState(const Quadrotor::State &state)
    {
        state_.x = state.x;
        state_.v = state.v;
        state_.rot = state.rot;
        state_.omega = state.omega;
        state_.motor_rpm = state.motor_rpm;
        state_.force_moment = state.force_moment;
        state_.timestamp = state.timestamp;
    }

    void Quadrotor::setStatePos(const Eigen::Vector3d &Pos)
    {
        state_.x = Pos;
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
        return external_moment_;
    }
    void Quadrotor::setExternalMoment(const Eigen::Vector3d &moment)
    {
        external_moment_ = moment;
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

    void Quadrotor::displaySetup()
    {
        axis_dist_ = 0.30;
        propeller_dist_ = 0.10;

        pangolin::CreateWindowAndBind("Model Predictive Control based on FGO", 1280, 960);

        // Define Camera Render Object (for view / scene browsing)
        s_cam = std::make_shared<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(1280, 960, 840, 840, 640, 480, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0.5, 1, 0, 0, 0, 0.0, -1.0, 0.0));

        // Choose a sensible left UI Panel width based on the width of 20
        // charectors from the default font.
        const int UI_WIDTH = 20 * pangolin::default_font().MaxWidth();

        d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, 640.0f / 480.0f)
                    .SetHandler(new pangolin::Handler3D(*s_cam));

        pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

        dis_force_ = std::make_shared<pangolin::Var<std::string>>("ui.Force(N)", "Force");
        dis_M1_ = std::make_shared<pangolin::Var<std::string>>("ui.M1(N*m)", "M1");
        dis_M2_ = std::make_shared<pangolin::Var<std::string>>("ui.M2(N*m)", "M2");
        dis_M3_ = std::make_shared<pangolin::Var<std::string>>("ui.M3(N*m)", "M3");
        dis_Quad_x_ = std::make_shared<pangolin::Var<std::string>>("ui.UAVx(m)", "UAVx");
        dis_Quad_y_ = std::make_shared<pangolin::Var<std::string>>("ui.UAVy(m)", "UAVy");
        dis_Quad_z_ = std::make_shared<pangolin::Var<std::string>>("ui.UAVz(m)", "UAVz");
        dis_Quad_velx_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vx(m/s)", "UAV_vx");
        dis_Quad_vely_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vy(m/s)", "UAV_vy");
        dis_Quad_velz_ = std::make_shared<pangolin::Var<std::string>>("ui.UAV_vz(m/s)", "UAV_vz");
        dis_AVE_ERR_ = std::make_shared<pangolin::Var<std::string>>("ui.AVE_ERR(m)", "AVE_ERR");
        dis_timestamp_ = std::make_shared<pangolin::Var<std::string>>("ui.TIMESTAMP(s)", "TIMESTAMP");
        dis_rotor_[0] = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR1(RPM)", "ROTOR1");
        dis_rotor_[1] = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR2(RPM)", "ROTOR2");
        dis_rotor_[2] = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR3(RPM)", "ROTOR3");
        dis_rotor_[3] = std::make_shared<pangolin::Var<std::string>>("ui.ROTOR4(RPM)", "ROTOR4");
    }

    void Quadrotor::drawQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot)
    {
        gtsam::Vector3 begin;
        gtsam::Vector3 end;
        begin = p;
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(gtsam::Vector3(0, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(gtsam::Vector3(0, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(gtsam::Vector3(0, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        drawLine(gtsam::Vector3(0, 0, 0), begin, end);
        drawFrame(p, rot);

        glColor3f(0, 0, 0);
        glPointSize(1.0);
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

    void Quadrotor::drawCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot)
    {
    }

    void Quadrotor::drawLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end)
    {
        glBegin(GL_LINES);
        glColor3f(color(0), color(1), color(2));
        glVertex3d(begin(0), begin(1), begin(2));
        glVertex3d(end(0), end(1), end(2));
        glEnd();
    }

    void Quadrotor::drawLidarCloud(Features &features)
    {
        glColor3f(0.1, 0.2, 0.7);
        glPointSize(5.0);
        glBegin(GL_POINTS);

        for (int idx = 0; idx < features.size(); idx++)
        {
            gtsam::Vector3 l_body_body(features[idx].x, features[idx].y, features[idx].z);
            gtsam::Vector3 l_body_w = state_.rot.rotate(l_body_body) + state_.x;
            glVertex3f(l_body_w.x(), l_body_w.y(), l_body_w.z());
        }

        glEnd();
    }

    void Quadrotor::drawFrame(gtsam::Vector3 p, gtsam::Rot3 rot)
    {
        gtsam::Vector3 begin = p;
        gtsam::Vector3 end;
        end = rot.rotate(gtsam::Vector3(0.03, 0, 0)) + begin;
        drawLine(gtsam::Vector3(1, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0.03, 0)) + begin;
        drawLine(gtsam::Vector3(0, 1, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0, 0.03)) + begin;
        drawLine(gtsam::Vector3(0, 0, 1), begin, end);
    }

    void Quadrotor::renderHistoryTrj()
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
                drawLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].x, trj_[i + 1].x);
            }
            drawQuadrotor(state_.x, state_.rot);

            last_state_.x = state_.x;

            renderPanel();

            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(100); // 0.1ms
        }
    }

    void Quadrotor::renderHistoryOpt(std::vector<State> &trj, boost::optional<gtsam::Vector3 &> err, boost::optional<Features &> features, 
        boost::optional<gtsam::Vector3&> vicon_measurement, boost::optional<gtsam::Vector3 &> rot_err, boost::optional<std::vector<State> &> state_trj)
    {
        gtsam::Vector3 error = *err;
        
        if(rot_err)
        {
            gtsam::Vector3 rot_error = *rot_err;
            record_info_ << state_.x[0] << " " << state_.x[1] << " " << state_.x[2] << " " << error[0] << " " << error[1] << " " << error[2] << " " << state_.force_moment[0] << " " 
            << state_.force_moment[1] << " " << state_.force_moment[2] << " " << state_.force_moment[3] << " " << 
            rot_error[0] << " " << rot_error[1] << " " << rot_error[2] << std::endl;
        }
        else
        {
            record_info_ << state_.x[0] << " " << state_.x[1] << " " << state_.x[2] << " " << error[0] << " " << error[1] << " " << error[2] << " " << state_.force_moment[0] << " " << state_.force_moment[1] << " " << state_.force_moment[2] << " " << state_.force_moment[3] << std::endl;
        }

        if (!pangolin::ShouldQuit())
        {
            // Clear screen and activate view to renderHistoryTrj into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if (trj_.size() > HISTORY_TRJ_LENS)
            {
                trj_.erase(trj_.begin());
            }

            trj_.push_back(state_);
            d_cam.Activate(*s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(1);
            drawFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());
            for (int i = 0; i < trj_.size() - 1; i++)
            {
                drawLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].x, trj_[i + 1].x);
            }

            glLineWidth(2);
            for (int i = 0; i < trj.size() - 1; i++)
            {
                drawLine(gtsam::Vector3(1.0, 0, 0), trj[i].x, trj[i + 1].x);
            }

            glLineWidth(2);
            if(state_trj)
            {
                int i;
                for (i = 0; i < (*state_trj).size() - 1; i++)
                {
                    drawLine(gtsam::Vector3(0.4f, 0.2f, 0.6f), (*state_trj)[i].x, (*state_trj)[i + 1].x);
                }
                drawLine(gtsam::Vector3(0.5f, 0.7f, 0.9f), (*state_trj)[i].x, trj[0].x);
            }

            drawQuadrotor(state_.x, state_.rot);

            last_state_.x = state_.x;

            if (errs_.size() >= ERRS_LENS)
            {
                errs_.erase(errs_.begin());
            }
            errs_.push_back(error);

            Features f = *features;
            drawLidarCloud(f);

            if(vicon_measurement)
            {
                glColor3f(0.6, 0.2, 0.5);
                glPointSize(10.0);
                glBegin(GL_POINTS);
                glVertex3f(vicon_measurement->x(), vicon_measurement->y(), vicon_measurement->z());
                glEnd();
                
            }
            
            for(uint i = 0; i < 314; i++)
            {
                glColor3f(0.3, 0.1, 0.8);
                glPointSize(2.0);
                glBegin(GL_POINTS);

                glVertex3f(1.5*sin(float(i)/ 314.0f * 6.28), 1.5*cos(float(i)/ 314.0f * 6.28), 1);
                
                glEnd();
            }

            renderPanel();

            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(1000);
        }
    }

    void Quadrotor::renderPanel()
    {
        std::string temp_str = std::to_string(state_.force_moment[0]);
        *dis_force_ = temp_str;
        std::stringstream ss;
        ss << std::setprecision(15) << state_.force_moment[1];
        *dis_M1_ = ss.str();
        ss << std::setprecision(15) << state_.force_moment[2];
        *dis_M2_ = ss.str();
        ss << std::setprecision(15) << state_.force_moment[3];
        *dis_M3_ = ss.str();
        temp_str = std::to_string(state_.x[0]);
        *dis_Quad_x_ = temp_str;
        temp_str = std::to_string(state_.x[1]);
        *dis_Quad_y_ = temp_str;
        temp_str = std::to_string(state_.x[2]);
        *dis_Quad_z_ = temp_str;
        temp_str = std::to_string(state_.v[0]);
        *dis_Quad_velx_ = temp_str;
        temp_str = std::to_string(state_.v[1]);
        *dis_Quad_vely_ = temp_str;
        temp_str = std::to_string(state_.v[2]);
        *dis_Quad_velz_ = temp_str;
        temp_str = std::to_string(state_.timestamp);
        *dis_timestamp_ = temp_str;
        temp_str = std::to_string(input_[0]);
        *dis_rotor_[0] = temp_str;
        temp_str = std::to_string(input_[1]);
        *dis_rotor_[1] = temp_str;
        temp_str = std::to_string(input_[2]);
        *dis_rotor_[2] = temp_str;
        temp_str = std::to_string(input_[3]);
        *dis_rotor_[3] = temp_str;

        double err_sum = 0;
        for (uint j = 0; j < errs_.size(); j++)
        {
            err_sum += errs_[j].transpose() * errs_[j];
        }
        double ave_err = std::sqrt(err_sum / errs_.size());
        temp_str = std::to_string(ave_err);
        *dis_AVE_ERR_ = temp_str;
    }

}
