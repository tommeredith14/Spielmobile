#include <ros/ros.h>

//C++ includes
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <stdint.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <queue>

//ROS includes
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <chrono>
//Other includes
#include "map.h"

class CSpielobileSimulator {
public:
	CSpielobileSimulator(std::string mapfile, double x = 1, double y = 1, double heading = 0) {
		m_pMap = new CMap(mapfile);
		m_robotLocation.linear.x = x;
		m_robotLocation.linear.y = y;
		m_robotLocation.linear.z = 0;
		m_robotLocation.angular.z = heading;


	    m_pNh = new ros::NodeHandle;
	    m_motionSub = m_pNh->subscribe<geometry_msgs::Twist>(
	    			"spielmobile/motion_updates", 10,
	    			&CSpielobileSimulator::MotionCallback, this);
	    m_locationPub = m_pNh->advertise<geometry_msgs::Twist>("spielmobile/current_location", 5);
	    m_scanPub = m_pNh->advertise<sensor_msgs::LaserScan>("/scan", 1);


	    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	    m_locationPub.publish(m_robotLocation);
	    m_pScannerThread = new std::thread(&CSpielobileSimulator::ScannerThread, this);
	    m_pSpinnerThread = new std::thread(&CSpielobileSimulator::SpinnerThread, this);
	}

	~CSpielobileSimulator() {
		if (m_pScannerThread)
			if (m_pScannerThread->joinable())
				m_pScannerThread->join();

		if (m_pSpinnerThread)
			if (m_pSpinnerThread->joinable())
				m_pSpinnerThread->join();
	}

	void MotionCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		UpdateLocation(msg);
		std::unique_lock<std::mutex> lock(m_locationMutex);
		m_locationPub.publish(m_robotLocation);
	}

	void SpinnerThread() {
		ros::spin();
	}
	void ScannerThread() {
		ros::Rate loop_rate(10);
		while (ros::ok()) {
			sensor_msgs::LaserScan scan;
			std::unique_lock<std::mutex> lock(m_locationMutex);
			m_pMap->SimulateScanFromPosition(scan, m_robotLocation);
			lock.unlock();
			
			m_scanPub.publish(scan);			

			loop_rate.sleep();
		}
	}
private:
	geometry_msgs::Twist m_robotLocation;
	std::mutex m_locationMutex;
	CMap* m_pMap;

	ros::Subscriber m_motionSub;
	ros::Publisher m_locationPub;
	ros::Publisher m_scanPub;
    ros::NodeHandle* m_pNh;

    std::thread* m_pScannerThread;
    std::thread* m_pSpinnerThread;

    void UpdateLocation(const geometry_msgs::Twist::ConstPtr& update) {
        double forward = update->linear.x;// + motionPosNoise(noise_generator);
	    double turnRad = update->linear.z;// + motionPosNoise(noise_generator);
	    double rotation = update->angular.z;// + motionHeadingNoise(noise_generator);
	    std::unique_lock<std::mutex> lock(m_locationMutex);
	    if (update->linear.x != 0)
	    {
	        m_robotLocation.linear.x += forward * cos(m_robotLocation.angular.z);
	        m_robotLocation.linear.y += forward * sin(m_robotLocation.angular.z);
	    }
	    else if (rotation > 0) {
	        double x_centre = m_robotLocation.linear.x - turnRad * sin(m_robotLocation.angular.z);
	        double y_centre = m_robotLocation.linear.y + turnRad * cos(m_robotLocation.angular.z);
	        m_robotLocation.linear.x = x_centre + turnRad * sin(m_robotLocation.angular.z + rotation);
	        m_robotLocation.linear.y = y_centre - turnRad * cos(m_robotLocation.angular.z + rotation);
	        m_robotLocation.angular.z += rotation;
	    } else {
	    	rotation = -rotation;
	        double x_centre = m_robotLocation.linear.x + turnRad * sin(m_robotLocation.angular.z);
	        double y_centre = m_robotLocation.linear.y - turnRad * cos(m_robotLocation.angular.z);
	        m_robotLocation.linear.x = x_centre - turnRad * sin(m_robotLocation.angular.z - rotation);
	        m_robotLocation.linear.y = y_centre + turnRad * cos(m_robotLocation.angular.z - rotation);
	        m_robotLocation.angular.z -= rotation;
	    }
    }

};


int main(int argc, char **argv)  {

	//Init ros
    ros::init(argc,argv, "spielmobile_simulator");
	CSpielobileSimulator simulator(argv[1]);


}