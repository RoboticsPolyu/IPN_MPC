#include <iostream>
#include <librealsense2/rs.hpp>

#include <yaml-cpp/yaml.h>

int main()
{
    YAML::Node RSM_config  = YAML::LoadFile("../config/rsm_uart.yaml");  
	std::uint16_t exposure_time = RSM_config["exposure_time"].as<std::uint16_t>();

    rs2::pipeline pipe;
    rs2::pipeline_profile prf = pipe.start();

    auto depthSensor = prf.get_device().query_sensors()[1];

    depthSensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    depthSensor.set_option(RS2_OPTION_EXPOSURE, 80);
}