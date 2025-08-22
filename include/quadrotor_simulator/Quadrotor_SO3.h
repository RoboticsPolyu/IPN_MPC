#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__

#include "color.h"
#include "Dynamics_factor.h"
#include "env_sensors_sim/Common.h"
#include "gtsam_wrapper.h"

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <vector>

#include "UI.h"

using namespace boost::numeric::odeint;
using namespace std;
using namespace UAVFactor;


namespace QuadrotorSim_SO3
{       
    class Quadrotor
    {
    public:
        using stateType = boost::array<double, 22>;
       // Dynamics related function

        Quadrotor();

        const State &getState(void) const;

        void setState(const State &state);

        void setStatePos(const Eigen::Vector3d &Pos);

        double getMass(void) const;
        void setMass(double mass);

        double getGravity(void) const;
        void setGravity(double g);

        const Eigen::Matrix3d &getInertia(void) const;
        void setInertia(const Eigen::Matrix3d &inertia);

        double getArmLength(void) const;
        void setArmLength(double d);

        double getPropRadius(void) const;
        void setPropRadius(double r);

        double getPropellerThrustCoefficient(void) const;
        void setPropellerThrustCoefficient(double kf);

        double getPropellerMomentCoefficient(void) const;
        void setPropellerMomentCoefficient(double km);

        double getMotorTimeConstant(void) const;
        void setMotorTimeConstant(double k);

        const Eigen::Vector3d &getExternalForce(void) const;
        void setExternalForce(const Eigen::Vector3d &force);

        const Eigen::Vector3d &getExternalMoment(void) const;
        void setExternalMoment(const Eigen::Vector3d &torque);

        double getMaxRPM(void) const;
        void setMaxRPM(double max_rpm);

        double getMinRPM(void) const;
        void setMinRPM(double min_rpm);

        // with 1 and 2 clockwise and 3 and 4 counter-clockwise (looking from top)
        void setInput(double u1, double u2, double u3, double u4);
        void setInput(gtsam::Vector4 thrust_torque);
        // Runs the actual dynamics simulation with a time step of dt
        void step(double dt);

        // ODE intergration based state propagation
        void operator()(const Quadrotor::stateType &x , Quadrotor::stateType &dxdt , const double t);
        void stepODE(double dt, gtsam::Vector4 fm);

        // Compute every rotor's rotation vel RPM
        Eigen::Vector4d CumputeRotorsVel();
        Eigen::Vector4d InvCumputeRotorsVel(Eigen::Vector4d rotor_speed);

        Eigen::Vector3d getAcc() const;
        
        void renderHistoryTrj()
        {
            ui_ptr->renderHistoryTrj();
        }

        void renderHistoryOpt(std::vector<State> & pred_trj, boost::optional<gtsam::Vector3&> err = boost::none,  
                                                        boost::optional<Features&> features = boost::none, 
                                                        boost::optional<gtsam::Vector3&> vicon_measurement = boost::none, 
                                                        boost::optional<gtsam::Vector3 &> rot_err = boost::none, 
                                                        boost::optional<std::vector<State> &> ref_trj = boost::none,
                                                        boost::optional<float &> opt_cost = boost::none);
        gtsam::Vector3 getObs1()
        {
            return ui_ptr->getObs1();
        }
        
        std::vector<Obstacle> getObsN()
        {
            for(uint8_t i = 0; i < obs_num_; i++)
            {
                obstacles_[i] = getObsbyEllipsev(i);
            }
            return obstacles_;
        }

    private:
        void printCurState();

        // Compute Control Allocation's effectiveness matrix
        Eigen::Matrix4d ComputeEffectivenessMatrix(); 
        
        // Control Allocation's effectiveness matrix
        Eigen::Matrix4d effectiveness_;

        gtsam::Vector3 getObsbyEllipse(uint8_t index);
        
        Obstacle getObsbyEllipsev(uint8_t index);         
        
        void initializeObstacles();

        void resolveCollisions();

        void updateObstaclePositions(double dt);

        double g_; // gravity
        double mass_;
        Eigen::Matrix3d J_; // Inertia
        double kf_;
        double km_;
        double prop_radius_;
        double arm_length_;
        double motor_time_constant_; // unit: sec
        double max_rpm_;
        double min_rpm_;
        double esc_factor_;
        
        Eigen::Vector3d drag_force_params_;

        State state_;
        
        float clock_ = 0.;
        float dt_ = 0.01;

        Eigen::Vector3d acc_;

        Eigen::Array4d  input_;
        Eigen::Vector3d external_force_;
        Eigen::Vector3d external_torque_;
        gtsam::Vector4  thrust_torque_;

        std::default_random_engine generator_;

        // force noise
        double THRUST_NOISE_MEAN  = 0.0;
        double THRUST_NOISE_COV   = 0.0;
        double ANGULAR_SPEED_MEAN = 0.0;
        double ANGULAR_SPEED_COV  = 0.0;

        Geometry geometry_;
        std::vector<Obstacle> obstacles_;
        std::vector<Obstacle> static_obstacles_;
        
        uint16_t obs_num_ = 0;
        std::shared_ptr<UI> ui_ptr;
    };
}
#endif
