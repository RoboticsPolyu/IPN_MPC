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
  double gt_t;
};

struct Battery_Data
{
  double battery_voltage;
  double gt_t;
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
    std::ifstream state_file, battery_file;
    std::string state_file_path = file_path + std::string("/rsm_2023-10-06-16-50-09.txt");
    std::string battery_file_path = file_path + std::string("/battery_rsm_2023-10-06-16-50-09.txt");
    state_file.open(state_file_path);
    battery_file.open(battery_file_path);


    double gt_t, battery_voltage, pwm, rotor_speed_sensor1, rotor_speed_sensor2;
    std::vector<Motor_Data> motor_datas;
    std::vector<Battery_Data> battery_datas;

    while (battery_file >> gt_t >> battery_voltage)
    {
      Battery_Data battery_data;
      battery_data.battery_voltage = battery_voltage;
      battery_data.gt_t = gt_t;
      battery_datas.push_back(battery_data);
      std::cout << battery_data.battery_voltage << " " << gt_t << std::endl;
    }


    uint32_t count = 0;
    double div = 1.0/16384.0*200.0*60.0;

    while (state_file >> gt_t >> rotor_speed_sensor1 >> rotor_speed_sensor2)
    {

      if(count > 4995 && count < 10500)
      {
        Motor_Data motor_data;
        motor_data.gt_t = gt_t;
        for(uint16_t bi = 0; bi < battery_datas.size(); bi++)
        {
          if(battery_datas[bi].gt_t <= gt_t && gt_t < battery_datas[bi+1].gt_t)
          {
            double interp = (gt_t - battery_datas[bi].gt_t)/ (battery_datas[bi+1].gt_t - battery_datas[bi].gt_t);
            battery_voltage = (1-interp)* battery_datas[bi].battery_voltage + interp* battery_datas[bi+1].battery_voltage;
          }
        }

        motor_data.battery_voltage = battery_voltage;
        motor_data.battery_voltage = 16;
        motor_data.rotor_speed1 = rotor_speed_sensor1* div;
        uint32_t count_add = count - 4995;
        motor_data.pwm = 1100 +  (count_add - count_add % 560) / 560 * 50;
        std::cout << motor_data.battery_voltage << " - " << motor_data.pwm << std::endl;
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