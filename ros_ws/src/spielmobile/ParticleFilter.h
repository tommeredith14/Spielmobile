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

	public:
		CParticle();
		CParticle(const CParticle& rhs);

	void MotionUpdate(const geometry_msgs::Twist::ConstPtr& update);
private:
	double x_pos;
	double y_pos;
	double heading;


};

class CParticleFilter {

public:
	CParticleFilter(int numParticles = 200);
	~CParticleFilter();
	void ProcessMotionUpdate(geometry_msgs::Twist::ConstPtr& update);
	void ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan);
	void SetMap(CMap* pmap);
	


private:
	std::vector<CParticle>* m_pParticleList;
	std::mutex m_mutexParticles
	CMap* m_pMap;


};

#endif //PARTICLEFILTER_H
