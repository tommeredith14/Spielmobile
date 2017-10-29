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
	friend class CParticleFilter;
public:
	CParticle();
	CParticle(const CParticle& rhs);

	void MotionUpdate(const geometry_msgs::Twist::ConstPtr& update);
	double CParticle::ComputeParticleProbability(sensor_msgs::LaserScan::ConstPtr& scan, CMap pMap);
private:
	double m_xpos;
	double m_ypos;
	double m_heading;
};



class CParticleFilter {

public:
	CParticleFilter(int numParticles = 200);
	~CParticleFilter();
	void ProcessMotionUpdate(geometry_msgs::Twist::ConstPtr& update);
	void ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan);
	void SetMap(CMap* pmap);
	


private:
	void ResampleParticles(std::vector<double>& vProbabilities);
	void NormalizeProbabilities(std::vector

	std::vector<CParticle>* m_pParticleList;
	std::mutex m_mutexParticles
	CMap* m_pMap;


};

#endif //PARTICLEFILTER_H
