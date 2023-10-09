#include <calibration/Calibration_factor.h>

#include <yaml-cpp/yaml.h>

using namespace gtsam;
using namespace std;
using namespace UAVFactor;


using symbol_shorthand::K; 

struct Motor_Data
{
  double rotor_speed1;
  double battery_voltage;
  double pwm;
};

int main(void)
{        
    // Configuration file 
    YAML::Node CAL_config = YAML::LoadFile("../config/motor_dynamics.yaml");

    auto rs_noise = noiseModel::Diagonal::Sigmas(Vector1(10));

    std::ofstream calib_log;
    std::string file_name = "../data/Motor/motor_calib_";
    file_name.append("debug");
    file_name.append("_log.txt");
    calib_log.open(file_name);
    
    std::string file_path = "../data/Motor";
    std::ifstream state_file;
    std::string state_file_path = file_path + std::string("/rsm_2023-10-06-16-50-09.txt");
    state_file.open(state_file_path);

    double gt_t, battery_voltage, pwm, rotor_speed_sensor1, rotor_speed_sensor2;
    std::vector<Motor_Data> motor_datas;

    uint32_t count = 0;
    while (state_file >> gt_t >> rotor_speed_sensor1 >> rotor_speed_sensor2)
    {

      if(count > 4995 && count < 10500)
      {
        std::cout << "gt: " << gt_t << std::endl;
        Motor_Data motor_data;
        motor_data.battery_voltage = 16;
        motor_data.rotor_speed1 = rotor_speed_sensor1/16384.0*200.0*60.0;
        uint32_t count_add = count - 4995;
        motor_data.pwm = 1100 +  (count_add - count_add % 560) / 560 * 50;
        // std::cout << motor_data.rotor_speed1 << " - " << motor_data.pwm << std::endl;
        motor_datas.push_back(motor_data);
      }
      count++;
    }

    uint32_t DATASET_S = 0;
    uint32_t DATASET_LENS = motor_datas.size();

    NonlinearFactorGraph dyn_factor_graph;
    Values initial_value_dyn;
    dyn_factor_graph.empty();

    for(uint32_t idx = 0; idx < 10; idx++)
    {   
      for(uint32_t nidx = 0; nidx < 30; nidx++)
      {
        uint32_t idxx = idx * 560 + nidx;

        Motor_Data data1 = motor_datas[idxx];
        Motor_Data data2 = motor_datas[idxx+1];

        MotorCalibFactor dynamicsCalibFactor(K(0), data1.battery_voltage, data1.pwm, data1.rotor_speed1, data2.rotor_speed1, 0.005, rs_noise);
        std::cout << data1.rotor_speed1 << " ----------- " << data1.pwm << std::endl;
        dyn_factor_graph.add(dynamicsCalibFactor);
      }
    }

    // for(uint32_t idx = DATASET_S; idx < DATASET_LENS; idx++)
    // {   
    //     Motor_Data data1 = motor_datas[idx];
    //     Motor_Data data2 = motor_datas[idx+1];

    //     // pose, velocity, angular speed, pose, velocity, angular speed, inertial of moments, rot of g, position of rotot, kf, km        
    //     MotorCalibFactor dynamicsCalibFactor(K(0), data1.battery_voltage, data1.pwm, data1.rotor_speed1, data2.rotor_speed1, 0.005, rs_noise);
    //     std::cout << data1.rotor_speed1 << " ----------- " << data1.pwm << std::endl;
    //     dyn_factor_graph.add(dynamicsCalibFactor);
    // }

    gtsam::Vector5 params;
    params << 1000, 0, 0, 1000, 0;
    initial_value_dyn.insert(K(0), params);

  
    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 1e-8;
    parameters.relativeErrorTol = 1e-8;
    parameters.maxIterations    = 500;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
    std::cout << "###################### init contoller optimizer ######################" << std::endl;
    LevenbergMarquardtOptimizer optimizer(dyn_factor_graph, initial_value_dyn, parameters);
    std::cout << "###################### begin optimize ######################" << std::endl;
    Values result = optimizer.optimize();
    
    params = result.at<gtsam::Vector5>(K(0));


    for(uint32_t idx = DATASET_S; idx < DATASET_LENS-1; idx++)
    {
      
      Motor_Data data1 = motor_datas[idx];
      Motor_Data data2 = motor_datas[idx+1];
      MotorCalibFactor dy(K(0), data1.battery_voltage, data1.pwm, data1.rotor_speed1, data2.rotor_speed1, 0.005, rs_noise);
      gtsam::Vector1 e = dy.evaluateError(params);
      calib_log << e << " " << data1.pwm << " " << data1.rotor_speed1 << " " << data2.rotor_speed1 - data1.rotor_speed1 << " " << std::endl;
    }

    std::cout << "S3:" << params.transpose() << "\n";

    return 0;
}