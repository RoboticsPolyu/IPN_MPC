#include "color.h"
#include "quadrotor_simulator/Quadrotor_SO3.h"

#include <ctime>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;


int main(int argc, char **argv)
{
    double dt = 0.001;
    QuadrotorSim_SO3::Quadrotor quad;

    QuadrotorSim_SO3::Quadrotor::State state = quad.getState();

    const double m = quad.getMass();
    const double g = quad.getGravity();
    const double kf = quad.getPropellerThrustCoefficient();

    const double hover_rpm = std::sqrt(m * g / (4 * kf));
    std::cerr << "hover rpm: " << hover_rpm << std::endl;
    state.motor_rpm = Eigen::Array4d(hover_rpm, hover_rpm, hover_rpm, hover_rpm);
    quad.setState(state);

    double thrust = m * g;
    double rpm = std::sqrt(1.05* thrust / (4 * kf));
    quad.setInput(rpm, rpm, rpm, rpm);

    struct timespec ts_start, ts1, ts2, ts_sleep, ts_end;
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    int64_t time_taken = 0;

    double KP = 8.0;
    double KD = 2.5;
    const double z_des = 0.5;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    
    std::cout.setf(ios::fixed);
    std::cout.setf(ios::right);
    std::cout.precision(3); //设置输出精度，保留有效数字
    std::cout.width(10);


    for (int i = 0; i < 4000; i++)
    {
        if(i % 100)
        {   
            quad.renderHistoryTrj();
        }
        
        state = quad.getState();
        thrust = m * g + KP * (z_des - state.p(2)) + KD * (0 - state.v(2));
        rpm = std::sqrt(thrust / (4 * kf));
        if (i < 3000)
            quad.setExternalForce(Eigen::Vector3d(0, 0, -KP * z_des));
        else
            quad.setExternalForce(Eigen::Vector3d(0, 0, 0));
            
        if(i < 3000)
        {
            quad.setInput(rpm, rpm, rpm, rpm);
        }
        else if(i >= 3000 & i < 3500)
        {
            quad.setInput(rpm, rpm, rpm* sqrt(1+ 0.01), rpm);
        }
        else
        {
            quad.setInput(rpm, rpm, rpm, rpm);
        }

        quad.step(dt);

        Eigen::Vector3d euler = state.rot.rpy();
        
        std::cout << i * dt << " s, " << "p: [" <<state.p << "], Eular: [" << euler(0) << ", " << euler(1) << ", " << euler(2) << "], " << state.body_rate(0) << ", " << state.body_rate(1) << ", " << state.body_rate(2) << ", " << state.motor_rpm(0) << std::endl;

        clock_gettime(CLOCK_MONOTONIC, &ts2);
        time_taken += ((ts2.tv_sec - ts1.tv_sec) * 1000000000UL + (ts2.tv_nsec - ts1.tv_nsec));
        int64_t time_sleep = i * dt / 2 * 1e9 - time_taken;
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        if (time_sleep > 0)
        {
            ts_sleep.tv_sec = time_sleep / 1000000000UL;
            ts_sleep.tv_nsec = time_sleep - ts_sleep.tv_sec * 1000000000UL;
            // nanosleep(&ts_sleep, NULL);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &ts_end);
    std::cerr << "Time: " << (ts_end.tv_sec - ts_start.tv_sec) * 1e6 + (ts_end.tv_nsec - ts_start.tv_nsec) / 1e3 << " usec" << std::endl;

    system("pause");
    return 0;
}
