#include "ParticleFilter.h"


#include <stdlib>
#include <time>
#include <cmath>
#include <random>

const double copyPosStdDev = 1.0;
const double copyHeadingStdDev = 1.0;
const double motionPosStdDev = 1.0;
const double copyHeadingStdDev = 1.0;

/****************************/
/* RANDOM GENERATORS */
/***************************/

std::default_random_engine noise_generator;
std::normal_distribution<double> copyPosNoise(0.0, copyPosStdDev);
std::normal_distribution<double> copyHeadingNoise(0.0, copyHeadingStdDev);
std::normal_distribution<double> motionPosNoise(0.0, motionPosStdDev);
std::normal_distribution<double> motionHeadingNoise(0.0, motionHeadingStdDev);



/****************************************************************/
/* PARTICLE */
/*****************************************************************/
CParticle::CParticle() {
	m_xpos = ((double)rand())/RAND_MAX*xMax;
	m_ypos = ((double)rand())/RAND_MAX*yMax;
	m_heading = ((double)rand())/RAND_MAX*2*M_PI;
}

CParticle::CParticle(const CParticle& rhs, bool randomize) {
	if (randomize) {
		m_heading = rhs.m_heading + copyHeadingNoise(noise_generator);
		m_xpos = rhs.m_xpos + copyPosNoise(noise_generator);
		m_ypos = rhs.m_ypos + copyPosNoise(noise_generator);
	} else {
		m_heading = rhs.m_heading;
		m_xpos = rhs.m_xpos;
		m_ypos = rhs.m_ypos;
	}
	
}

void CParticle::MotionUpdate(const geometry_msgs::Twist::ConstPtr& update) {
	double forward = update->linear.x + motionPosNoise(noise_generator);
	double turnRad = update->linear.z + motionPosNoise(noise_generator);
	double rotation = update->angular.z + motionHeadingNoise(noise_generator);
	
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

double CParticle::ComputeParticleProbability(sensor_msgs::LaserScan::ConstPtr& scan, CMap pMap) {

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
		
		for (auto &particle : (*m_pParticleList))
		{
			particle.MotionUpdate(update);
		}
		particleLock.unlock();
}


void CParticleFilter::ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan) {
	std::vector<double> probabilityList(m_pParticleList.size());
	//calculate probability of each particle
	int particleNum = 0;
	for (auto &particle: (*m_pParticleList)) {
		probabilityList[particleNum] = particle.ComputeParticleProbability(scan, m_pMap);
		particleNum++;
	}
	
	//normalize probabilities
	
	//re-sample particles
	

}


void CParticleFilter::SetMap(CMap* pMap) {
	m_pMap = pMap;
}
