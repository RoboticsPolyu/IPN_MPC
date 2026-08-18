#ifndef __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__
#define __QUADROTOR_SIMULATOR_QUADROTOR_SO3_H__

#include <Eigen/Core>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <fstream>
#include <iostream>
#include <ipn_mpc/factors/gtsam_compatibility.h>
#include <ipn_mpc/simulation/imu.h>
#include <ipn_mpc/simulation/types.h>
#include <ipn_mpc/visualization/ui.h>
#include <yaml-cpp/yaml.h>
#include <vector>

using namespace boost::numeric::odeint;
using namespace std;

namespace QuadrotorSim_SO3 {
class Quadrotor {
  public:
    using stateType = boost::array<double, 22>;
    // Dynamics related function

    Quadrotor();

    Quadrotor(const std::string& yaml_file, bool enable_visualization = true);

    const State& getState(void) const;

    void setState(const State& state);

    void setStatePos(const Eigen::Vector3d& Pos);

    double getMass(void) const;
    void setMass(double mass);

    double getGravity(void) const;
    void setGravity(double g);

    const Eigen::Matrix3d& getInertia(void) const;
    void setInertia(const Eigen::Matrix3d& inertia);

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

    const Eigen::Vector3d& getExternalForce(void) const;
    void setExternalForce(const Eigen::Vector3d& force);

    const Eigen::Vector3d& getExternalMoment(void) const;
    void setExternalMoment(const Eigen::Vector3d& torque);

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
    void operator()(const Quadrotor::stateType& x, Quadrotor::stateType& dxdt, const double t);
    void stepODE(double dt, gtsam::Vector4 fm);

    // Compute every rotor's rotation vel RPM
    Eigen::Vector4d computeRotorVelocities();
    Eigen::Vector4d inverseRotorVelocities(Eigen::Vector4d rotor_speed);

    Eigen::Vector3d getAcc() const;
    const IMUMeasurement& getIMUMeasurement() const { return latest_imu_measurement_; }

    bool renderHistoryTrj() {
        return ui_ptr ? ui_ptr->renderHistoryTrj(state_) : true;
    }

    bool renderHistoryOpt(std::vector<State>& pred_trj,
                          boost::optional<gtsam::Vector3&> err = boost::none,
                          boost::optional<Features&> features = boost::none,
                          boost::optional<gtsam::Vector3&> vicon_measurement = boost::none,
                          boost::optional<gtsam::Vector3&> rot_err = boost::none,
                          boost::optional<std::vector<State>&> ref_trj = boost::none,
                          boost::optional<float&> opt_cost = boost::none);
    gtsam::Vector3 getObs1() {
        return ui_ptr ? ui_ptr->getObs1() : gtsam::Vector3::Zero();
    }

    std::vector<Obstacle> getObstacles() {
        updateObstaclePositions(clock_);
        return obstacles_;
    }

    void setPerformanceStats(double solve_time_ms, double mean_solve_time_ms,
                             double p95_solve_time_ms, double cpu_percent,
                             std::size_t deadline_misses) {
        if (ui_ptr)
            ui_ptr->setPerformanceStats(solve_time_ms, mean_solve_time_ms, p95_solve_time_ms,
                                        cpu_percent, deadline_misses);
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
    void loadObstacles(const YAML::Node& config);
    void configureIMU(const YAML::Node& config);
    void updateIMUMeasurement(double dt);

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

    Eigen::Vector3d acc_{Eigen::Vector3d::Zero()};
    Sensors_Sim::IMU imu_;
    IMUMeasurement latest_imu_measurement_;

    Eigen::Array4d input_;
    Eigen::Vector3d external_force_;
    Eigen::Vector3d external_torque_;
    gtsam::Vector4 thrust_torque_;

    std::default_random_engine generator_;

    // force noise
    double thrust_noise_mean = 0.0;
    double thrust_noise_cov = 0.0;
    double angular_speed_mean = 0.0;
    double angular_speed_cov = 0.0;

    std::vector<Obstacle> obstacles_;
    std::vector<Obstacle> static_obstacles_;

    uint16_t obs_num_ = 0;
    uint16_t cylinder_num_ = 0;
    uint16_t static_obs_num_ = 0;
    double sphere_radius_ = 0;
    double quad_size_ = 0;
    std::shared_ptr<UI> ui_ptr;
};
} // namespace QuadrotorSim_SO3
#endif
