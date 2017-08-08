#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <stdint.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;

#define INCREMENT 0.01

ros::Publisher pub;
bool done_snapshot = false;
bool snapshot_started = false;

sensor_msgs::LaserScan scan360;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	//Do something

}


int main(int argc, char **argv) {
	//Init ros
	ros::init(argc,argv, "Lidar_snapshotter");
	ros::NodeHandle nh;

	//Create publisher, set message type
	pub = nh.advertise<geometry_msgs::Twist>("spielmobile/Lidar_snapshots", 10);

	ros::Rate rate(1);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10,scanCallback);

	scan360.angle_min = 0;
	scan360.angle_max = M_PI*2;
	scan_msg.angle_increment = INCREMENT;
	

	ros::spin();
	return 0;

}
