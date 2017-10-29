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
	double tru
	
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
