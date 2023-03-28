#include "quadrotor_simulator/Quadrotor_SO3.h"
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <iostream>

#define RED   (1.0,0.0,0.0)
#define GREEN (0.0,1.0,0.0)
#define BLUE  (0.0,0.0,1.0)

namespace QuadrotorSimulator_SO3
{

    void Quadrotor::setup()
    {
        axis_dist_ = 0.30;
        propeller_dist_ = 0.10;
        
        pangolin::CreateWindowAndBind("Main", 640, 480);
        //glEnable(GL_DEPTH_TEST);
        //glEnable(GL_BLEND);
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Define Camera Render Object (for view / scene browsing)
        s_cam = std::make_shared<pangolin::OpenGlRenderState>(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
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
        pLine(gtsam::Vector3(0,0,0), begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0,0,0), begin, end);
        end = rot.rotate(gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0,0,0), begin, end);
        end = rot.rotate(gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0)) + p;
        pLine(gtsam::Vector3(0,0,0), begin, end);
        pFrame(p, rot);

        pCircle(gtsam::Vector3(0,0,0), prop_radius_, gtsam::Vector3(axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0),
        gtsam::Rot3::identity());
        pCircle(gtsam::Vector3(0,0,0), prop_radius_, gtsam::Vector3(-axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0),
        gtsam::Rot3::identity());
        pCircle(gtsam::Vector3(0,0,0), prop_radius_, gtsam::Vector3(-axis_dist_ / 2 / 1.414, axis_dist_ / 2 / 1.414, 0),
        gtsam::Rot3::identity());
        pCircle(gtsam::Vector3(0,0,0), prop_radius_, gtsam::Vector3(axis_dist_ / 2 / 1.414, -axis_dist_ / 2 / 1.414, 0),
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
        pLine(gtsam::Vector3(1,0,0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0.03, 0)) + begin;
        pLine(gtsam::Vector3(0,1,0), begin, end);
        end = rot.rotate(gtsam::Vector3(0, 0, 0.03)) + begin;
        pLine(gtsam::Vector3(0,0,1), begin, end);
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
            pFrame(gtsam::Vector3(0,0,0), gtsam::Rot3::identity());
            for(int i = 0; i < trj_.size() -1; i++)
            {
                pLine(gtsam::Vector3(0.5,0,0.5), trj_[i].x, trj_[i+1].x);
            }
            pQuadrotor(state_.x, state_.rot);

            last_state_.x = state_.x;

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

        Eigen::Array4d motor_rpm_dot;
        Eigen::Vector3d vnorm;
        Eigen::Array4d motor_rpm_sq;

        motor_rpm_sq = state_.motor_rpm.array().square();

        double thrust = kf_ * motor_rpm_sq.sum();

        Eigen::Vector3d moments;
        moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
        moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
        moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
                            motor_rpm_sq(3));

        double resistance = 0.1 *                                        // C
                            3.14159265 * (arm_length_) * (arm_length_) * // S
                            state_.v.norm() * state_.v.norm();

        //  ROS_INFO("resistance: %lf, Thrust: %lf%% ", resistance,
        //           motor_rpm_sq.sum() / (4 * max_rpm_ * max_rpm_) * 100.0);

        vnorm = state_.v;
        if (vnorm.norm() != 0)
        {
            vnorm.normalize();
        }

        Eigen::Vector3d v_dot = -Eigen::Vector3d(0, 0, g_) +
                                state_.rot.rotate(gtsam::Vector3(0, 0, thrust)) / mass_ +
                                external_force_ / mass_; // - resistance * vnorm / mass_;
        Eigen::Vector3d p_dot = state_.v;
        
        // J* omega_dot = moments - J.cross(J* omega)
        Eigen::Vector3d omega_dot =
            J_.inverse() *
            (moments - state_.omega.cross(J_ * state_.omega) + external_moment_);
        motor_rpm_dot = (input_ - state_.motor_rpm) / dt; 

        // Predict state
        predicted_state_.x = state_.x + p_dot * dt;
        predicted_state_.v = state_.v + v_dot * dt;
        predicted_state_.rot = state_.rot * gtsam::Rot3::Expmap(state_.omega * dt);
        predicted_state_.omega = state_.omega + omega_dot * dt;
        predicted_state_.motor_rpm = state_.motor_rpm + motor_rpm_dot * dt;

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
