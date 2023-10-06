#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <IPN_MPC/Rsm.h>
#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
	YAML::Node RSM_config  = YAML::LoadFile("../config/rsm_uart.yaml");  
    std::string dev_name   = RSM_config["dev_name"].as<std::string>();
	std::string topic_name = RSM_config["topic_name"].as<std::string>();

	ros::init(argc, argv, "RotorSpeedNode");

    ros::NodeHandle node;

    ros::Publisher rotor_rsm_pub = node.advertise<IPN_MPC::Rsm>(topic_name, 1000);

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
	IPN_MPC::Rsm rsm_msg;
	uint16_t rotor1_speed = 0; 
	uint16_t rotor2_speed = 0;
	uint16_t rotor3_speed = 0;
	uint16_t rotor4_speed = 0;

	while(true)
	{
		nbytes = read(s, &frame, sizeof(struct can_frame));

		if (nbytes < 0) 
		{
			perror("Read");
			return 1;
		}

		if(frame.can_id = 0x11)
		{
			rotor1_speed = frame.data[1];rotor1_speed |= frame.data[0] << 8;
			rotor2_speed = frame.data[3];rotor1_speed |= frame.data[2] << 8;
			rotor3_speed = frame.data[5];rotor1_speed |= frame.data[4] << 8;
			rotor4_speed = frame.data[7];rotor1_speed |= frame.data[6] << 8;

			ROS_DEBUG("Rotor1 speed: %d\r\n", rotor1_speed);
			ROS_DEBUG("Rotor2 speed: %d\r\n", rotor2_speed);
			ROS_DEBUG("Rotor3 speed: %d\r\n", rotor3_speed);
			ROS_DEBUG("Rotor4 speed: %d\r\n", rotor4_speed);

			rsm_msg.header.seq   = count;
			rsm_msg.header.stamp = ros::Time::now();
			rsm_msg.rotor1_speed = rotor1_speed;
			rsm_msg.rotor2_speed = rotor2_speed;
			rsm_msg.rotor3_speed = rotor3_speed;
			rsm_msg.rotor4_speed = rotor4_speed;
			rotor_rsm_pub.publish(rsm_msg);
			count++;
		}
		else if(frame.can_id = 0x12)
		{

		}
		ros::spinOnce();

	}

	if (close(s) < 0) {
		perror("Close");
		return 1;
	}

	return 0;
}
