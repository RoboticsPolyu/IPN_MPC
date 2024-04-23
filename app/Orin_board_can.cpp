#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <IPN_MPC/Rsm.h>
#include <IPN_MPC/IMU.h>
#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <yaml-cpp/yaml.h>

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
                return -(65536 - input)*1000.0/32768.0;
        }
        else
        {
                return input*1000.0/32768.0;

        }
}


int main(int argc, char **argv)
{
	YAML::Node RSM_config  = YAML::LoadFile("../config/rsm_uart.yaml");  
  std::string dev_name   = RSM_config["dev_name"].as<std::string>();
	std::string rsm_topic_name = RSM_config["rsm_topic_name"].as<std::string>();
	std::string imu_topic_name = RSM_config["imu_topic_name"].as<std::string>();

	ros::init(argc, argv, "RotorSpeedNode");

  ros::NodeHandle node;

  ros::Publisher rotor_rsm_pub = node.advertise<IPN_MPC::Rsm>(rsm_topic_name, 1000);
	ros::Publisher rotor_imu_pub = node.advertise<IPN_MPC::IMU>(imu_topic_name, 1000);

	int s; 
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	printf("CAN Sockets Receive APP\r\n");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

    int count = 0;
	int count_imu = 0;

	IPN_MPC::Rsm rsm_msg;
	IPN_MPC::IMU imu_msg;

	uint16_t rotor1_speed = 0; 
	uint16_t rotor2_speed = 0;
	uint16_t rotor3_speed = 0;
	uint16_t rotor4_speed = 0;

	uint16_t acc_x = 0, acc_y = 0, acc_z = 0, acc_id = 0;
	uint16_t gyro_x = 0, gyro_y = 0, gyro_z = 0, gyro_id = 0;

	while(ros::ok())
	{
		nbytes = read(s, &frame, sizeof(struct can_frame));

		if (nbytes < 0) 
		{
			perror("Read");
			return 1;
		}

		if(frame.can_id == 0x11) // 100hz
		{
			rotor1_speed = frame.data[1];rotor1_speed |= frame.data[0] << 8;
			rotor2_speed = frame.data[3];rotor2_speed |= frame.data[2] << 8;
			rotor3_speed = frame.data[5];rotor3_speed |= frame.data[4] << 8;
			rotor4_speed = frame.data[7];rotor4_speed |= frame.data[6] << 8;

			ROS_DEBUG("Rotor1 speed: %d\r\n", rotor1_speed);
			ROS_DEBUG("Rotor2 speed: %d\r\n", rotor2_speed);
			ROS_DEBUG("Rotor3 speed: %d\r\n", rotor3_speed);
			ROS_DEBUG("Rotor4 speed: %d\r\n", rotor4_speed);

			if(count % 100 == 0)
			{
				ROS_INFO("Rotor speed - [%d] - [%d] - [%d] - [%d] ", rotor1_speed, rotor2_speed, rotor3_speed, rotor4_speed);
			}
			rsm_msg.header.seq   = count;
			rsm_msg.header.stamp = ros::Time::now();
			rsm_msg.rotor1_speed = rotor1_speed;
			rsm_msg.rotor2_speed = rotor2_speed;
			rsm_msg.rotor3_speed = rotor3_speed;
			rsm_msg.rotor4_speed = rotor4_speed;
			rotor_rsm_pub.publish(rsm_msg);
			count++;
		}
		else if(frame.can_id == 0x12) // 200hz
		{
			acc_x = frame.data[1]; acc_x |= frame.data[0] << 8;
			acc_y = frame.data[3]; acc_y |= frame.data[2] << 8;
			acc_z = frame.data[5]; acc_z |= frame.data[4] << 8;
			acc_id = frame.data[6];

			ROS_DEBUG("ACC X: %d\r\n", acc_x);
			ROS_DEBUG("ACC Y: %d\r\n", acc_y);
			ROS_DEBUG("ACC Z: %d\r\n", acc_z);
			ROS_DEBUG("ACC ID: %d\r\n", acc_id);

		}
		else if(frame.can_id == 0x13) // 200hz
		{
			gyro_x = frame.data[1]; gyro_x |= frame.data[0] << 8;
			gyro_y = frame.data[3]; gyro_y |= frame.data[2] << 8;
			gyro_z = frame.data[5]; gyro_z |= frame.data[4] << 8;
			gyro_id = frame.data[6];

			ROS_DEBUG("GYRO X: %d\r\n", gyro_x);
			ROS_DEBUG("GYRO Y: %d\r\n", gyro_y);
			ROS_DEBUG("GYRO Z: %d\r\n", gyro_z);
			ROS_DEBUG("GYRO ID: %d\r\n", gyro_id);

			if(count_imu % 1000 == 0)
			{
				float acc1, acc2, acc3, gy1, gy2, gy3;

				ROS_INFO("Acc - [%f] - [%f] - [%f] - Gyro - [%f] - [%f] - [%f]", convert_acc(acc_x), convert_acc(acc_y), convert_acc(acc_z), convert_gyro(gyro_x), convert_gyro(gyro_y), convert_gyro(gyro_z) );
			}

			imu_msg.header.seq   = count;
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.acc_x   = convert_acc(acc_x);
			imu_msg.acc_y   = convert_acc(acc_y);
			imu_msg.acc_z   = convert_acc(acc_z);
			imu_msg.acc_id  = acc_id;
			imu_msg.gyro_x  = convert_gyro(gyro_x);
			imu_msg.gyro_y  = convert_gyro(gyro_y);
			imu_msg.gyro_z  = convert_gyro(gyro_z);
			imu_msg.gyro_id = gyro_id;

			rotor_imu_pub.publish(imu_msg);

			count_imu++;

		}
		ros::spinOnce();

	}

	if (close(s) < 0) 
	{
		perror("Close");
		return 1;
	}

	return 0;
}
