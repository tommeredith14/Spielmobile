#include "ParticleFilter.h"


#include <stdlib>
#include <time>

/*****************************************************************/
/* PARTICLE */
/*****************************************************************/
CParticle::CParticle() {

}

CParticle::CParticle(const CParticle& rhs) {

}

void CParticle::MotionUpdate(const geometry_msgs::Twist::ConstPtr& update) {
	double forward = update->linear.x;
	double turnRad = update->linear.z;
	double rotation = update->angular.z
	
	if (update->linear.x != 0)
	{
		(*this).m_xpos += forward * cos(m_heading);
		(*this).m_ypos += forward * sin(heading);
	}
	else if (rotation > 0) {
		double x_centre = (*this).m_xpos - turnRad * sin((*this).heading);
		double y_centre = (*this).m_ypos + turnRad * cos((*this).heading);
		(*this).m_xpos = x_centre + turnRad * sin((*this).heading + rotation);
		(*this).m_ypos = y_centre - turnRad * cos((*this).heading + rotation);
		(*this).heading += rotation;
	} else {
		double x_centre = (*this).m_xpos + turnRad * sin((*this).heading);
		double y_centre = (*this).m_ypos - turnRad * cos((*this).heading);
		(*this).m_xpos = x_centre - turnRad * sin((*this).heading - rotation);
		(*this).m_ypos = y_centre + turnRad * cos((*this).heading - rotation);
		(*this).heading -= rotation;
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
