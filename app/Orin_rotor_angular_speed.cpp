#include <hardware/spi_driver.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"



int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	mode |= SPI_CPHA;

	char address_angle[4] = "\xFF\xFF";
	input_tx = address_angle;

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	if (input_tx && input_file)
		pabort("only one of -p and --input may be selected");


	ros::init(argc, argv, "MotorEncoder");

	ros::NodeHandle n;

    ros::Publisher rotor_av_pub = n.advertise<std_msgs::UInt16>("/Quad13/Dynamics/MotorEncoder1", 1000);
 
    ros::Rate loop_rate(2000);

    int count = 0;

	size_t size = strlen(input_tx);
	uint8_t *tx;
	uint8_t *rx;

	tx = (uint8_t*) malloc(size);
	if (!tx)
		pabort("can't allocate tx buffer");

	rx = (uint8_t*) malloc(size);
	if (!rx)
		pabort("can't allocate rx buffer");

	size = unescape((char *)tx, input_tx, size);

	uint16_t rotor_angular_vel = 0;
	uint16_t motor_ang = 0;

    while (ros::ok())
    {
    	std_msgs::UInt16 msg;
     	// msg.data = rotor_angular_vel;
		msg.data = motor_ang;
	
		transfer(fd, tx, rx, size);
		motor_ang = ((((uint16_t) rx[0] << 8) | rx[1]) & 0x3fff);

     	// ROS_INFO("The Rotor's Angular Speed - %u", motor_ang);

     	rotor_av_pub.publish(msg);

     	ros::spinOnce();
 
    	loop_rate.sleep();
     	++count;
    }
	
	free(rx);
	free(tx);
	close(fd);
	return ret;
}