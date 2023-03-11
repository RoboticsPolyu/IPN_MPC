#include "wio_factor.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>

using namespace wio;

using namespace std;
using namespace gtsam;

using symbol_shorthand::B;
using symbol_shorthand::P;
using symbol_shorthand::R;
using symbol_shorthand::V;
using symbol_shorthand::X;

int main(void)
{
    std::cout << "###################### congratulation ######################" << std::endl;

    const double kGravity = 9.81;
    auto params = PreintegrationParams::MakeSharedU(kGravity);
    params->setAccelerometerCovariance(I_3x3 * 1e-3);
    params->setGyroscopeCovariance(I_3x3 * 1e-4);
    params->setIntegrationCovariance(I_3x3 * 1e-8);
    params->setUse2ndOrderCoriolis(false);
    params->setOmegaCoriolis(Vector3(0, 0, 0));

    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    Vector3 wheel_speed_noise(0.01, 0, 0);
    Matrix3 wheel_cov = wheel_speed_noise.asDiagonal();

    gtsam::Rot3 bRo = Rot3::RzRyRx(0, 0, 0); // 0 0 0
    Vector3 bPo(0, 1, 0);                    // real 0 1 0

    PreintegratedImuWheelMeasurements i_w_pim(params, prior_imu_bias, bRo, wheel_cov);

    Vector3 position, acc, gyro, imu_velocity, wheel_speed;
    vector<Vector3> positions, accs, gyros, imu_velocitys, wheel_speeds;
    Pose3 pose_bak;
    Vector3 vel_bak;
    Pose3 pose_0;
    Vector3 vel_0;

    Quaternion imu_quat;
    vector<Quaternion> imu_quats;
    double dt = 0.01f;
    Values initial_value;
    ifstream imu_file;
    imu_file.open("../data/imu_pose_noise.txt");

    double t, qw, qx, qy, qz, t0, t1, t2, v0, v1, v2, g0, g1, g2, a0, a1, a2, wv;
    uint64_t j = 0, key = 0;

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));

    while (imu_file >> t >> qw >> qx >> qy >> qz >> t0 >> t1 >> t2 >> v0 >> v1 >> v2 >> g0 >> g1 >> g2 >> a0 >> a1 >> a2 >> wv)
    {
        acc << a0, a1, a2;
        gyro << g0, g1, g2;
        imu_velocity << v0, v1, v2;
        position << t0, t1, t2;
        imu_quat = Quaternion(qw, qx, qy, qz);
        wheel_speed << wv, 0, 0;
        accs.push_back(acc);
        gyros.push_back(gyro);
        imu_velocitys.push_back(imu_velocity);
        positions.push_back(position);
        imu_quats.push_back(imu_quat);
        wheel_speeds.push_back(wheel_speed);
    }

    for (uint32_t index_t = 0; index_t < accs.size(); index_t++)
    {
        acc = accs[index_t];
        gyro = gyros[index_t];
        imu_velocity = imu_velocitys[index_t];
        position = positions[index_t];
        imu_quat = imu_quats[index_t];
        wheel_speed = wheel_speeds[index_t];

        Vector3 position_next = positions[index_t + 1];
        Quaternion imu_quat_next = imu_quats[index_t + 1];
        Vector3 imu_velocity_next = imu_velocitys[index_t];
        Pose3 pose_next = Pose3(Rot3(imu_quat_next.toRotationMatrix()), position_next);

        if (j == 0)
        {
            pose_0 = Pose3(Rot3(imu_quat.toRotationMatrix()), position);
            vel_0 = imu_velocity;
        }

        // std::cout << "j: " << j << " ,acc: " << acc << " ,gyro: " << gyro << " ,imu_vel: " << imu_velocity << " ,pos: " << position << " ,ws: " << wheel_speed << std::endl;
        if (j % 10 == 0)
        {
            pose_bak = Pose3(Rot3(imu_quat.toRotationMatrix()), position);
            vel_bak = imu_velocity;
        }

        i_w_pim.integrateMeasurement(acc, gyro, wheel_speed, dt);
        if (j % 10 == 9)
        {
            WheelImuFactor wheel_imu_factor(X(key), V(key), X(key + 1), V(key + 1), B(key), R(0), P(0), i_w_pim);

            Matrix jac_posei, jac_posej, jac_bias;
            Matrix jac_veli, jac_velj, jac_r, jac_p;
            std::cout << "******************* evalute error wrp bPo *******************" << std::endl;

            pose_bak = Pose3(Rot3(imu_quat.toRotationMatrix()), position);
            vel_bak = imu_velocity;

            // i_w_pim.resetIntegration();
            key++;
            if (key == 5)
            {
                for (int32_t i = 0; i < 256; i++)
                {

                    bPo << 0, (float)i / 128, 0;
                    Vector12 error = wheel_imu_factor.evaluateError(pose_0, vel_0, pose_next, imu_velocity_next, zero_bias, bRo, bPo, jac_posei, jac_veli, jac_posej, jac_velj, jac_bias, jac_r, jac_p);
                    std::cout << "bpo: \n"
                              << bPo << "\n error: \n"
                              << error.tail(3).norm() << std::endl;
                };

                for (int32_t i = -128; i < 128; i++)
                {

                    bPo << (float)i / 128, 1, 0;
                    Vector12 error = wheel_imu_factor.evaluateError(pose_0, vel_0, pose_next, imu_velocity_next, zero_bias, bRo, bPo, jac_posei, jac_veli, jac_posej, jac_velj, jac_bias, jac_r, jac_p);
                    std::cout << "bpo: \n"
                              << bPo << "\n error: \n"
                              << error.tail(3).norm() << std::endl;
                };
                return 0;
            };
        }
        j++;
        std::cout << "------------- j: " << j << " -------------key: " << key << std::endl;
    }

    return 0;
}