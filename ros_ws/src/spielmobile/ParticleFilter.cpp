#include "ParticleFilter.h"


#include <stdlib>
#include <time>
#include <cmath>

const double copyStdDev = 1.0;
const double motionStdDev = 1.0;

/*****************************************************************/
/* PARTICLE */
/*****************************************************************/
CParticle::CParticle() {
	m_xpos = ((double)rand())/RAND_MAX*xMax;
	m_ypos = ((double)rand())/RAND_MAX*yMax;
	m_heading = ((double)rand())/RAND_MAX*2*M_PI;
}

CParticle::CParticle(const CParticle& rhs, bool randomize) {
	if (randomize) {
	
	} else {
		m_heading = rhs.m_heading
		m_xpos = rhs.m_xpos;
		m_ypos = rhs.m_ypos;
	}
	
}

void CParticle::MotionUpdate(const geometry_msgs::Twist::ConstPtr& update) {
	double forward = update->linear.x;
	double turnRad = update->linear.z;
	double rotation = update->angular.z
	
	if (update->linear.x != 0)
	{
		m_xpos += forward * cos(m_heading);
		m_ypos += forward * sin(m_heading);
	}
	else if (rotation > 0) {
		double x_centre = m_xpos - turnRad * sin(m_heading);
		double y_centre = m_ypos + turnRad * cos(m_heading);
		m_xpos = x_centre + turnRad * sin(m_heading + rotation);
		m_ypos = y_centre - turnRad * cos(m_heading + rotation);
		m_heading += rotation;
	} else {
		double x_centre = m_xpos + turnRad * sin(m_heading);
		double y_centre = m_ypos - turnRad * cos(m_heading);
		m_xpos = x_centre - turnRad * sin(m_heading - rotation);
		m_ypos = y_centre + turnRad * cos(m_heading - rotation);
		m_heading -= rotation;
	}


}


/*****************************************************************/
/* PARTICLE FILTER */
/*****************************************************************/


CParticleFilter::CParticleFilter(int numParticles) {
	srand(time(nullptr));
	m_pParticleList = new std::vector<CParticle>(numParticles);

}

CParticleFilter::~CParticleFilter() {
	delete m_pParticleList;l
	m_pParticleList = nullptr;
}


void CParticleFilter::ProcessMotionUpdate(geometry_msgs::Twist::ConstPtr& update) {
		//Update the particles
		std::unique_lock<std::mutex> partlicleLock(mutexParticles);
		
		for (auto &particle : particleList)
		{
			particle.MotionUpdate(update);
		}
		particleLock.unlock();
}


void CParticleFilter::ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan) {

}


void CParticleFilter::SetMap(CMap* pmap) {
	m_pMap = pMap;
}
