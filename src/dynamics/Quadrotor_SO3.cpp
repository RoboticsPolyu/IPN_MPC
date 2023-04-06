#include "quadrotor_simulator/Quadrotor_SO3.h"
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

namespace QuadrotorSimulator_SO3
{

    void Quadrotor::setup()
    {
        axis_dist_ = 0.30;
        propeller_dist_ = 0.10;

        pangolin::CreateWindowAndBind("Main", 1280, 960);
        // glEnable(GL_DEPTH_TEST);
        // glEnable(GL_BLEND);
        // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
    }

    void Quadrotor::pQuadrotor(gtsam::Vector3 p, gtsam::Rot3 rot)
    {
        gtsam::Vector3 begin;
        gtsam::Vector3 end;
        begin = p;
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0, 0, 0), begin, end);
        pFrame(p, rot);

        pCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
        pCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
        pCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
        pCircle(gtsam::Vector3(0, 0, 0), prop_radius_, gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0),
                gtsam::Rot3::identity());
    }

    void Quadrotor::pCircle(gtsam::Vector3 color, float r, gtsam::Vector3 p, gtsam::Rot3 rot)
    {
    }

    void Quadrotor::pLine(gtsam::Vector3 color, gtsam::Vector3 begin, gtsam::Vector3 end)
    {
        glBegin(GL_LINES);
        glColor3f(color(0), color(1), color(2));
        glVertex3d(begin(0), begin(1), begin(2));
        glVertex3d(end(0), end(1), end(2));
        glEnd();
    }

    void Quadrotor::pFrame(gtsam::Vector3 p, gtsam::Rot3 rot)
    {
        gtsam::Vector3 begin = p;
        gtsam::Vector3 end;
        end = rot.rotate(gtsam::Vector3(0.03, 0, 0)) + begin;
        pLine(gtsam::Vector3(1, 0, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0.03, 0)) + begin;
        pLine(gtsam::Vector3(0, 1, 0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0, 0.03)) + begin;
        pLine(gtsam::Vector3(0, 0, 1), begin, end);
    }

    void Quadrotor::render()
    {
        if (!pangolin::ShouldQuit())
        {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            trj_.push_back(state_);

            d_cam.Activate(*s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(2);
            pFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());
            for (int i = 0; i < trj_.size() - 1; i++)
            {
                pLine(gtsam::Vector3(0.5, 0, 0.5), trj_[i].x, trj_[i + 1].x);
            }
            pQuadrotor(state_.x, state_.rot);

            last_state_.x = state_.x;

            pangolin::default_font().Text("UAV").Draw(state_.x.x(), state_.x.y(), state_.x.z());

            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(10);
        }
    }

    void Quadrotor::render_test(std::vector<State> &trj)
    {
        if (!pangolin::ShouldQuit())
        {
            // Clear screen and activate view to render into
            // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(*s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glLineWidth(2);
            pFrame(gtsam::Vector3(0, 0, 0), gtsam::Rot3::identity());
            for (int i = 0; i < trj.size() - 1; i++)
            {
                pLine(gtsam::Vector3(0.5, 0, 0.5), trj[i].x, trj[i + 1].x);
            }

            if (trj.size() > 0)
            {
                pQuadrotor(trj[trj.size() - 1].x, trj[trj.size() - 1].rot);
            }

            pangolin::default_font().Text("UAV").Draw(state_.x.x(), state_.x.y(), state_.x.z());

            // Swap frames and Process Events
            pangolin::FinishFrame();
            usleep(10);
        }
    }

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

        arm_length_ = 0.26;
        motor_time_constant_ = 1.0 / 30;
        min_rpm_ = 1200;
        max_rpm_ = 35000;

        drag_force_p = Eigen::Vector3d::Zero();

        state_.x = Eigen::Vector3d::Zero();     // position
        state_.v = Eigen::Vector3d::Zero();     // velocity
        state_.rot = gtsam::Rot3::identity();   // altitude
        state_.omega = Eigen::Vector3d::Zero(); // angular velocity
        state_.motor_rpm = Eigen::Array4d::Zero();

        last_state_ = state_;

        external_force_.setZero();

        input_ = Eigen::Array4d::Zero();

        setup();
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

        double resistance = 0.1 *                                        // C
                            3.14159265 * (arm_length_) * (arm_length_) * // S
                            state_.v.norm() * state_.v.norm();

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -state_.rot.matrix() * Eigen::Matrix3d(drag_force_p.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_ + drag_force;
        // std::cout << "vdot: " << v_dot << std::endl;
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

        // std::cout << "ode: " << cur_state.x.transpose() << std::endl;

        double thrust = x[18]; // cur force
        // std::cout << "thrust: " << thrust << std::endl;
        Eigen::Vector3d moment(x[19], x[20], x[21]); // cur moment

        // std::cout << "moments: " << moment.transpose() << std::endl;

        double resistance = 0.1 *                                        // C
                            3.14159265 * (arm_length_) * (arm_length_) * // S
                            cur_state.v.norm() * cur_state.v.norm();

        vnorm = cur_state.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -cur_state.rot.matrix() * Eigen::Matrix3d(drag_force_p.asDiagonal()) * cur_state.rot.matrix().transpose() * cur_state.v;
        Eigen::Vector3d v_dot = - Eigen::Vector3d(0, 0, g_) + cur_state.rot.rotate(gtsam::Vector3(0, 0, thrust / mass_)) +
                                external_force_ / mass_ + drag_force;
        // std::cout << "vdot: " << v_dot << std::endl;
        Eigen::Vector3d p_dot = cur_state.v;
        // std::cout << "pdot: " << p_dot << std::endl;
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

        for(int i = 0; i < 4; i++)
        {
            dxdt[18 +i] = (force_moment_[i] - state_.force_moment[i]) / 0.01;
        }
        // std::cout << "dxdt18-21: " << (force_moment_ - state_.force_moment).transpose() / 0.01 << std::endl;

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

        for(int i = 0; i < 4; i++)
        {
            x[18 +i] = state_.force_moment[i];
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

        for(int i = 0; i < 4; i++)
        {
            state_.force_moment[i] = x[18 + i];
        }

        // std::cout << "force_moment diff: " << force_moment_ - state_.force_moment << std::endl;
        // Re-orthonormalize R (polar decomposition)
        Eigen::LLT<Eigen::Matrix3d> llt(cur_rotm.transpose() * cur_rotm);
        Eigen::Matrix3d P = llt.matrixL();
        Eigen::Matrix3d R = cur_rotm * P.inverse();

        state_.rot = gtsam::Rot3(R);

        // noise
        std::normal_distribution<double> x_noise(0.0, 0.05);
        std::normal_distribution<double> r_noise(0.0, 0.005);
        std::normal_distribution<double> v_noise(0.0, 0.005);
        std::normal_distribution<double> o_noise(0.0, 0.005);

        gtsam::Vector3 pos_noise = dt* gtsam::Vector3(x_noise(generator_), x_noise(generator_), x_noise(generator_));
        std::cout << "pos_noise: " << pos_noise.transpose() << std::endl;
        gtsam::Vector3 vel_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(v_noise(generator_), v_noise(generator_), v_noise(generator_));
        std::cout << "vel_noise: " << vel_noise.transpose() << std::endl;
        gtsam::Vector3 rot_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(r_noise(generator_), r_noise(generator_), r_noise(generator_));
        gtsam::Vector3 ome_noise = dt * gtsam::Vector3(o_noise(generator_), o_noise(generator_), o_noise(generator_));

        
        state_.x = state_.x + pos_noise;
        state_.v = state_.v + vel_noise;
        
        printCurState();
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

    void Quadrotor::step_fm(double dt, gtsam::Vector4 fm)
    {
        State predicted_state_;

        Eigen::Vector3d vnorm;

        double thrust = fm[0];

        Eigen::Vector3d moments = fm.tail(3);

        double resistance = 0.1 *                                        // C
                            3.14159265 * (arm_length_) * (arm_length_) * // S
                            state_.v.norm() * state_.v.norm();

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -state_.rot.matrix() * Eigen::Matrix3d(drag_force_p.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_ + drag_force;
        // std::cout << "vdot: " << v_dot << std::endl;
        Eigen::Vector3d p_dot = state_.v;
        Eigen::Matrix3d r_dot = state_.rot.matrix() * gtsam::skewSymmetric(state_.omega);

        // J* omega_dot = moments - J.cross(J* omega)
        Eigen::Vector3d omega_dot = J_.inverse() * (moments - state_.omega.cross(J_ * state_.omega) + external_moment_);

        // noise
        std::normal_distribution<double> x_noise(0.0, 0.05);
        std::normal_distribution<double> r_noise(0.0, 0.005);
        std::normal_distribution<double> v_noise(0.0, 0.005);
        std::normal_distribution<double> o_noise(0.0, 0.005);

        gtsam::Vector3 pos_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(x_noise(generator_), x_noise(generator_), x_noise(generator_));
        gtsam::Vector3 vel_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(v_noise(generator_), v_noise(generator_), v_noise(generator_));
        std::cout << "vel_noise: " << vel_noise.transpose() << std::endl;
        gtsam::Vector3 rot_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(r_noise(generator_), r_noise(generator_), r_noise(generator_));
        gtsam::Vector3 ome_noise = dt * gtsam::Vector3(o_noise(generator_), o_noise(generator_), o_noise(generator_));

        std::cout << "pos_noise: " << pos_noise.transpose() << std::endl;
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

    void Quadrotor::step_noise(double dt)
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

        double resistance = 0.1 *                                        // C
                            3.14159265 * (arm_length_) * (arm_length_) * // S
                            state_.v.norm() * state_.v.norm();

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d drag_force = -state_.rot.matrix() * Eigen::Matrix3d(drag_force_p.asDiagonal()) * state_.rot.matrix().transpose() * state_.v;
        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) + state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_ + drag_force;
        // std::cout << "vdot: " << v_dot << std::endl;
        Eigen::Vector3d p_dot = state_.v;

        // J* omega_dot = moments - J.cross(J* omega)
        Eigen::Vector3d omega_dot = J_.inverse() * (moments - state_.omega.cross(J_ * state_.omega) + external_moment_);

        // noise
        std::normal_distribution<double> x_noise(0.0, 0.05);
        std::normal_distribution<double> r_noise(0.0, 0.005);
        std::normal_distribution<double> v_noise(0.0, 0.005);
        std::normal_distribution<double> o_noise(0.0, 0.005);

        // Predict state
        double dt22 = 0.5 * dt * dt;
        gtsam::Vector3 pos_noise = dt * gtsam::Vector3(x_noise(generator_), x_noise(generator_), x_noise(generator_));
        gtsam::Vector3 vel_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(v_noise(generator_), v_noise(generator_), v_noise(generator_));
        std::cout << "vel_noise: " << vel_noise.transpose() << std::endl;
        gtsam::Vector3 rot_noise = gtsam::Vector3::Zero(); // dt* gtsam::Vector3(r_noise(generator_), r_noise(generator_), r_noise(generator_));
        gtsam::Vector3 ome_noise = dt * gtsam::Vector3(o_noise(generator_), o_noise(generator_), o_noise(generator_));

        std::cout << "pos_noise: " << pos_noise.transpose() << std::endl;
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

    const Eigen::Vector3d &
    Quadrotor::getExternalForce(void) const
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
}
