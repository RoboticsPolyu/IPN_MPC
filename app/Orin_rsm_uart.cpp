#include <fcntl.h>
#include <hardware/uart.h>
#include <inttypes.h>
#include <IPN_MPC/Rsm.h>
#include <IPN_MPC/IMU.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include <string.h>
#include <termios.h>
#include <yaml-cpp/yaml.h>

#define MAX_READ_SIZE 9
#define G 9.81

float convert_acc(uint16_t input)
{
	if(input > 32768)
	{
		return -(65536 - input)*8*G/32768;
	}
	else
	{
		return input*8*G/32768;
	}
}



float convert_gyro(uint16_t input)
{
	if(input > 32768)
	{
		return -(65536 - input)*1000.0/32768.0/180.0*M_PI;
	}
	else
	{
		return input*1000.0/32768.0/180.0*M_PI;
	}
}

int main(int argc, char *argv[]) 
{
	YAML::Node RSM_config  = YAML::LoadFile("../config/rsm_uart.yaml");  
	std::string dev_name   = RSM_config["dev_name"].as<std::string>();
	std::string rsm_topic_name = RSM_config["rsm_topic_name"].as<std::string>();
	std::string imu_topic_name = RSM_config["imu_topic_name"].as<std::string>();

	ros::init(argc, argv, "RotorSpeedNode");

    ros::NodeHandle node;

    ros::Publisher rotor_rsm_pub = node.advertise<IPN_MPC::Rsm>(rsm_topic_name, 1000);
	ros::Publisher rotor_imu_pub = node.advertise<sensor_msgs::Imu>(imu_topic_name, 1000);

	struct UartDevice dev;
	int rc;

	dev.filename = new char[dev_name.length() + 1];
	strcpy(dev.filename, dev_name.c_str());
	dev.rate = B921600;
	ROS_INFO("Start uart !\r\n");
	rc = uart_start(&dev, false);
	if (rc) 
	{
		return rc;
	}

	char read_data[MAX_READ_SIZE];
	size_t read_data_len;

	ROS_INFO("This is rotor speed node program !\r\n");
	uart_writes(&dev, "QUAD13\r\n");
	ros::Duration(0.5).sleep();

	// The increment of contigous frame
	// The count value of one circle is 2^14
	uint16_t rotor1_speed = 0; 
	uint16_t rotor2_speed = 0;
	uint16_t rotor3_speed = 0;
	uint16_t rotor4_speed = 0;

	uint16_t acc_x = 0, acc_y = 0, acc_z = 0, acc_id = 0;
	uint16_t gyro_x = 0, gyro_y = 0, gyro_z = 0, gyro_id = 0;

    // ros::Rate loop_rate(2000);

  int count = 0;
	int count_imu = 0;

	IPN_MPC::Rsm rsm_msg;
	// IPN_MPC::IMU imu_msg;
	sensor_msgs::Imu	imu_msg;

	while (ros::ok())
	{
		read_data_len = uart_reads(&dev, read_data, MAX_READ_SIZE);

		// ROS_INFO("%d\r\n", read_data_len);
		
		if (read_data_len >= 8) 
		{
			// ROS_INFO("%d\r\n", read_data_len);
			
			uint8_t flag = (uint8_t)read_data[6];

			
			if(flag == 0x40)
			{
				memcpy((char*)&acc_x, (char*)read_data+0, sizeof(acc_x));
				memcpy((char*)&acc_y, (char*)read_data+2, sizeof(acc_y));
				memcpy((char*)&acc_z, (char*)read_data+4, sizeof(acc_z));
				// ROS_INFO("ACC x: %f\r\n", convert_acc(acc_x) );
				// ROS_INFO("ACC y: %f\r\n", convert_acc(acc_y) );
				// ROS_INFO("ACC z: %f\r\n", convert_acc(acc_z) );
				
			}
			else if(flag == 0x80)
			{
				memcpy((char*)&gyro_x, (char*)read_data+0, sizeof(gyro_x));
				memcpy((char*)&gyro_y, (char*)read_data+2, sizeof(gyro_y));
				memcpy((char*)&gyro_z, (char*)read_data+4, sizeof(gyro_z));

				// ROS_INFO("GYRO x: %f\r\n", convert_gyro(gyro_x) );
				// ROS_INFO("GYRO y: %f\r\n", convert_gyro(gyro_y) );
				// ROS_INFO("GYRO z: %f\r\n", convert_gyro(gyro_z) );
					
				imu_msg.header.seq   = count_imu;
				imu_msg.header.stamp = ros::Time::now();

				imu_msg.linear_acceleration.x = convert_acc(acc_x);
				imu_msg.linear_acceleration.y = convert_acc(acc_y);
				imu_msg.linear_acceleration.z = convert_acc(acc_z);
				
				imu_msg.angular_velocity.x = convert_gyro(gyro_x);
				imu_msg.angular_velocity.y = convert_gyro(gyro_y);
        		imu_msg.angular_velocity.z = convert_gyro(gyro_z);

				rotor_imu_pub.publish(imu_msg);

				count_imu++;

				if(count_imu % 1000 == 0)
				{
					float acc1, acc2, acc3, gy1, gy2, gy3;

				        //	ROS_INFO("Acc - [%f] - [%f] - [%f] - Gyro - [%f] - [%f] - [%f]", convert_acc(acc_x), convert_acc(acc_y), convert_acc(acc_z), convert_gyro(gyro_x), convert_gyro(gyro_y), convert_gyro(gyro_z) );
					ROS_INFO("Rotor speed: [%d] - [%d] - [%d] - [%d]", rotor1_speed, rotor2_speed, rotor3_speed, rotor4_speed);
				}


			}
			else
			{
				memcpy((char*)&rotor1_speed, (char*)read_data+0, sizeof(rotor1_speed));
				memcpy((char*)&rotor2_speed, (char*)read_data+2, sizeof(rotor2_speed));
				memcpy((char*)&rotor3_speed, (char*)read_data+4, sizeof(rotor3_speed));
				memcpy((char*)&rotor4_speed, (char*)read_data+6, sizeof(rotor4_speed));

			
				// ROS_INFO("Rotor1 speed: %d\r\n", rotor1_speed);
				// ROS_INFO("Rotor2 speed: %d\r\n", rotor2_speed);
				// ROS_INFO("Rotor3 speed: %d\r\n", rotor3_speed);
				// ROS_INFO("Rotor4 speed: %d\r\n", rotor4_speed);
				
				rsm_msg.header.seq   = count;
				rsm_msg.header.stamp = ros::Time::now();
				rsm_msg.rotor1_speed = rotor1_speed;
				rsm_msg.rotor2_speed = rotor2_speed;
				rsm_msg.rotor3_speed = rotor3_speed;
				rsm_msg.rotor4_speed = rotor4_speed;
				rotor_rsm_pub.publish(rsm_msg);
				count++;

			}
			// rsm_msg.header.seq = count;
			// rsm_msg.header.stamp = ros::Time::now();
			// rsm_msg.rotor1_speed = rotor1_speed;
			// rsm_msg.rotor2_speed = rotor2_speed;
			// rsm_msg.rotor3_speed = rotor3_speed;
			// rsm_msg.rotor4_speed = rotor4_speed;
			// rotor_rsm_pub.publish(rsm_msg);
			// count++;
			ros::spinOnce();

			// loop_rate.sleep();
		}
	}

	uart_stop(&dev);

  return 0;
}
