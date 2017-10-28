#include "ParticleFilter.h"

CParticleFilter::CParticleFilter(int numParticles) {


}

CParticleFilter::~CParticleFilter() {

}


void CParticleFilter::ProcessMotionUpdate(geometry_msgs::Twist::ConstPtr& update);
void CParticleFilter::ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan);
void SetMap(CMap* pmap);
