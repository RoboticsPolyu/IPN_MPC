#include <fcntl.h>
#include <gflags/gflags.h>
#include "glog/logging.h"
#include <hardware/uart.h>
#include <inttypes.h>
#include <IPN_MPC/Rsm.h>
#include <ros/console.h> 
#include "ros/ros.h"
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <string.h>
#include <termios.h>

#define MAX_READ_SIZE 9

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "RotorSpeedNode");

    ros::NodeHandle node;

    ros::Publisher rotor_rsm_pub = node.advertise<IPN_MPC::Rsm>("/Quad13/Dynamics/RotorSpeed", 1000);

	struct UartDevice dev;
	int rc;

	dev.filename = "/dev/ttyUSB0";
	dev.rate = B115200;

	rc = uart_start(&dev, false);
	if (rc) 
	{
		return rc;
	}

	char read_data[MAX_READ_SIZE];
	size_t read_data_len;

	ROS_INFO("This is rotor speed node program !\r\n");
	uart_writes(&dev, "QUAD13\r\n");
	ros::Duration(1.0).sleep();

	uint16_t rotor1_speed = 0;
	uint16_t rotor2_speed = 0;
	uint16_t rotor3_speed = 0;
	uint16_t rotor4_speed = 0;

    // ros::Rate loop_rate(2000);

    int count = 0;
	
	IPN_MPC::Rsm rsm_msg;

	while (ros::ok())
	{
		read_data_len = uart_reads(&dev, read_data, MAX_READ_SIZE);

		if (read_data_len > 0) 
		{
			ROS_DEBUG("%d\r\n", read_data_len);
			memcpy((char*)&rotor1_speed, (char*)read_data+0, sizeof(rotor1_speed));
			memcpy((char*)&rotor2_speed, (char*)read_data+2, sizeof(rotor2_speed));
			memcpy((char*)&rotor3_speed, (char*)read_data+4, sizeof(rotor3_speed));
			memcpy((char*)&rotor4_speed, (char*)read_data+6, sizeof(rotor4_speed));
			ROS_DEBUG("Rotor1 speed: %d\r\n", rotor1_speed);
			ROS_DEBUG("Rotor2 speed: %d\r\n", rotor2_speed);
			ROS_DEBUG("Rotor3 speed: %d\r\n", rotor3_speed);
			ROS_DEBUG("Rotor4 speed: %d\r\n", rotor4_speed);

			rsm_msg.header.seq = count;
			rsm_msg.header.stamp = ros::Time::now();
			rsm_msg.rotor1_speed = rotor1_speed;
			rsm_msg.rotor2_speed = rotor2_speed;
			rsm_msg.rotor3_speed = rotor3_speed;
			rsm_msg.rotor4_speed = rotor4_speed;
			rotor_rsm_pub.publish(rsm_msg);
			count++;
			ros::spinOnce();

			// loop_rate.sleep();
		}
	}

	uart_stop(&dev);

    return 0;
}
