#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

//std includes
#include <vector>


//ROS includes
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>

//Other includes
#include "map.h"


class CParticle {
	private:
		double x_pos;
		double y_pos;
		double heading;

	public:
		CParticle() {

		}

		void MotionUpdate(const geometry_msgs::Twist::ConstPtr& update) {
			
			if (update->linear.x != 0)
			{
				(*this).x_pos += update->linear.x * cos(heading);
				(*this).y_pos += update->linear.x * sin(heading);
			}
			else if (update->angular.z > 0) {
				double x_centre = (*this).x_pos - update->linear.z * sin((*this).heading);
				double y_centre = (*this).y_pos + update->linear.z * cos((*this).heading);
				(*this).x_pos = x_centre + update->linear.z * sin((*this).heading + update->angular.z);
				(*this).y_pos = y_centre - update->linear.z * cos((*this).heading + update->angular.z);
				(*this).heading += update->angular.z;
			} else {
				double x_centre = (*this).x_pos + update->linear.z * sin((*this).heading);
				double y_centre = (*this).y_pos - update->linear.z * cos((*this).heading);
				(*this).x_pos = x_centre - update->linear.z * sin((*this).heading - update->angular.z);
				(*this).y_pos = y_centre + update->linear.z * cos((*this).heading - update->angular.z);
				(*this).heading -= update->angular.z;
			}


		}


};

class CParticleFilter {

public:
	void ProcessMotionUpdate(geometry_msgs::Twist::ConstPtr& update);
	void ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan);
	void SetMap(CMap* map);
	


private:
	std::vector<CParticle> m_particleList;
	std::mutex m_mutexParticles
	CMap* pMap;


};

#endif //PARTICLEFILTER_H
