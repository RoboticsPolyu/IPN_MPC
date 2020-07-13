#include "wheel_imu_factor.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>

#include <fstream>

using namespace relate_factor;

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

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations = 500;
    parameters.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM = gtsam::LevenbergMarquardtParams::SUMMARY;

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
    // Vector3 rot_xyz(0, 0.2, 0.3);
    // Quaternion rot_b_o_q = Quaternion(0.996613, 0, 0.043584, 0.0697343); // 0 5deg 8deg
    gtsam::Rot3 bRo = Rot3::RzRyRx(0, 0, 0); // 0 0 0
    // gtsam::Rot3 bRo = Rot3(rot_b_o_q.toRotationMatrix());
    Vector3 bPo(0, 1, 0); // real 0 1 0

    PreintegratedImuWheelMeasurements i_w_pim(params, prior_imu_bias, bRo, wheel_cov);
    NonlinearFactorGraph graph;
    Vector3 position, acc, gyro, imu_velocity, wheel_speed;
    Pose3 pose_bak;
    Vector3 vel_bak;
    Pose3 pose_0;
    Vector3 vel_0;

    Quaternion imu_quat;
    double dt = 0.01f;
    Values initial_value;
    ifstream imu_file;
    imu_file.open("imu_pose.txt");

    double t, qw, qx, qy, qz, t0, t1, t2, v0, v1, v2, g0, g1, g2, a0, a1, a2, wv;
    uint64_t j = 0, key = 0;

    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001));
    auto wheelexttonoise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 100, 0.001));
    auto wheelextronoise = noiseModel::Diagonal::Sigmas(Vector3(0.001, 100, 100));
    auto posnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.001));
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(1e-6));

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));

    while (imu_file >> t >> qw >> qx >> qy >> qz >> t0 >> t1 >> t2 >> v0 >> v1 >> v2 >> g0 >> g1 >> g2 >> a0 >> a1 >> a2 >> wv)
    {
        acc << a0, a1, a2;
        gyro << g0, g1, g2;
        imu_velocity << v0, v1, v2;
        position << t0, t1, t2;
        imu_quat = Quaternion(qw, qx, qy, qz);
        wheel_speed << wv, 0, 0;

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
            graph.addPrior(X(key), pose_bak, posnoise);
            graph.addPrior(V(key), vel_bak, velnoise);
            graph.addPrior(B(key), zero_bias, biasnoise);
            initial_value.insert(X(key), pose_bak);
            initial_value.insert(V(key), vel_bak);
            initial_value.insert(B(key), zero_bias);

            WheelImuFactor wheel_imu_factor(X(key), V(key), X(key + 1), V(key + 1), B(key), R(0), P(0), i_w_pim);

            Pose3 pose_end = Pose3(Rot3(imu_quat.toRotationMatrix()), position);
            Vector3 vel_end = imu_velocity;

            Matrix jac_posei, jac_posej, jac_bias;
            Matrix jac_veli, jac_velj, jac_r, jac_p;

            Vector12 error = wheel_imu_factor.evaluateError(pose_0, vel_0, pose_end, vel_end, zero_bias, bRo, bPo, jac_posei, jac_veli, jac_posej, jac_velj, jac_bias, jac_r, jac_p);

            std::cout << "******************* evalute error *******************" << std::endl;
            std::cout << "error: \n"
                      << error << std::endl;
            std::cout << "jac posei: \n"
                      << jac_posei << std::endl;
            std::cout << "jac posej: \n"
                      << jac_posej << std::endl;
            std::cout << "jac veli: \n"
                      << jac_veli << std::endl;
            std::cout << "jac velj: \n"
                      << jac_velj << std::endl;
            std::cout << "jac bias: \n"
                      << jac_bias << std::endl;
            std::cout << "jac r: \n"
                      << jac_r << std::endl;
            std::cout << "jac p: \n"
                      << jac_p << std::endl;
            std::cout << "**************************************" << std::endl;

            // WheelImuFactor2 wheel_imu_factor(X(key), V(key), X(key + 1), V(key + 1), B(key), R(0), i_w_pim, bPo);
            graph.add(wheel_imu_factor);
            if (j != 499) //499
            {
                graph.add(BetweenFactor<imuBias::ConstantBias>(B(key), B(key + 1), zero_bias, biasnoise));
            }
            // i_w_pim.resetIntegration();
            key++;
            if (key == 50)
            {
                return 0;
            };
        }
        j++;
        std::cout << "------------- j: " << j << " -------------key: " << key << std::endl;
    }

    Pose3 pose_end = Pose3(Rot3(imu_quat.toRotationMatrix()), position);
    Vector3 vel_end = imu_velocity;
    initial_value.insert(X(key), pose_end);
    initial_value.insert(V(key), vel_end);

    imu_file.close();

    initial_value.insert(R(0), bRo);
    initial_value.insert(P(0), bPo);
    graph.addPrior(P(0), bPo, wheelexttonoise);
    graph.addPrior(R(0), bRo, wheelextronoise);

    std::cout << "###################### init optimizer ######################" << std::endl;
    LevenbergMarquardtOptimizer optimizer(graph, initial_value, parameters);

    std::cout << "###################### begin optimize ######################" << std::endl;
    Values result = optimizer.optimize();
    // result.print("Final Result:\n");
    Rot3 optbRo = result.at<Rot3>(R(0));
    Vector3 optbPo = result.at<Vector3>(P(0));

    std::cout << "optbRo: \n"
              << Rot3::LocalCoordinates(optbRo) << std::endl;
    std::cout << "optbPo: \n"
              << optbPo << std::endl;
    return 0;
}