#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <linux/i2c.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>

using namespace std;

int file_i2c;

int main(int argc, char **argv) {
	//Init ros
	ros::init(argc,argv, "ARM_comms");
	ros::NodeHandle nh;

	//Create publisher, set message type
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("spielmobile/motion_updates", 100);

	int fd, result;
	char *filename = (char*)"/dev/i2c-1";
	
	file_i2c = open(filename, O_RDWR);
	if (file_i2c < 0)
		cout << "Init failed\n";



	ros::Rate rate(1);

	while(ros::ok()) {

		geometry_msgs::Twist msg;


		if (ioctl(file_i2c, I2C_SLAVE, 11) < 0) {
			cout << "FAILED ioctl";
		}
		/////////
		uint8_t to_send[2] = {0,0};

		int16_t input;
		cout << "Left wheel: ";
		cin >> input;
		to_send[0]= (uint8_t)input;

		cin.clear();
		cin.ignore(99999, '\n');

		cout << "Right wheel: ";
		cin >> input;
		to_send[1]= (uint8_t)input;

		cout << "sending: " << (int)to_send[0] << ", " << (int)to_send[1] << endl;
		uint8_t to_read[6] = {0};

		if (write(file_i2c,to_send,2) != 2) {
			cout << "failed write\n";
			goto exit;
		}
			
		usleep(1000);

		if (read(file_i2c, to_read,6) != 6) {
			cout << "failed read";
			goto exit;
		}
		for (int i = 0; i < 6; i++)
			cout << (uint16_t)to_read[i] << endl;
		cin.clear();
		cin.ignore(99999, '\n');

		//Decipher the values
		// 00SF_XXXX XXXX_XXXX XXXX_XXXX
		// -->
		// xxxx_xxxx xxxx_xx00 0011_1111 1222_2222
		int left_dist = 0;
		int right_dist = 0;

		left_dist |= ((to_read[0] & 0x0F) << 10);
		left_dist |= ((to_read[1] & 0x7F) << 7);
		left_dist |= ((to_read[2] & 0x7F) << 0);
		if ((to_read[0] & 0x20) != 0)
			left_dist *= -1;

		right_dist |= ((to_read[3] & 0x0F) << 10);
		right_dist |= ((to_read[4] & 0x7F) << 7);
		right_dist |= ((to_read[5] & 0x7F) << 0);
		if ((to_read[3] & 0x20) != 0)
			right_dist *= -1;


		cout << "Left wheel went  " << left_dist << endl;
		cout << "Right wheel went " << right_dist << endl;






		msg.linear.x = right_dist;
		msg.linear.y = left_dist;

		pub.publish(msg);

//			usleep(100000);
		rate.sleep();


	}
	return 0;
	exit:
		close(file_i2c);


}
