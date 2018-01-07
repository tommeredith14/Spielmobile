#include "ParticleFilter.h"


#include <cstdlib>
#include <ctime>
#include <cmath>
#include <random>
#include <limits>
#include <iostream>
#include <chrono>

const double copyPosStdDev = 0.0;
const double copyHeadingStdDev = 1.0;
const double motionPosStdDev = 0.1;
const double motionHeadingStdDev = 0.1;

/****************************/
/* RANDOM GENERATORS */
/***************************/

std::default_random_engine noise_generator;
std::normal_distribution<double> copyPosNoise(0.0, copyPosStdDev);
std::normal_distribution<double> copyHeadingNoise(0.0, copyHeadingStdDev);
std::normal_distribution<double> motionPosNoise(1.0, motionPosStdDev);
std::normal_distribution<double> motionHeadingNoise(1.0, motionHeadingStdDev);

std::uniform_real_distribution<double> xUniformDistribution;
std::uniform_real_distribution<double> yUniformDistribution;
std::uniform_real_distribution<double> headingUniformDistribution(0.0, 2*M_PI);
std::uniform_real_distribution<double> resamplerDistribution;
std::uniform_int_distribution<int> randomIndex;

/****************************************************************/
/* PARTICLE */
/*****************************************************************/
int xMax;//////////////////////
int yMax;/////////////////////
CParticle::CParticle() {
    x = xUniformDistribution(noise_generator);
    y = yUniformDistribution(noise_generator);
    heading = headingUniformDistribution(noise_generator);
}

CParticle::CParticle(const CParticle& rhs, bool randomize) {
    if (randomize) {
        heading = rhs.heading + copyHeadingNoise(noise_generator);
        x = rhs.x + copyPosNoise(noise_generator);
        y = rhs.y + copyPosNoise(noise_generator);
    } else {
        heading = rhs.heading;
        x = rhs.x;
        y = rhs.y;
    }
    
}

void CParticle::MotionUpdate(const geometry_msgs::Twist::ConstPtr& update) {
    double forward = update->linear.x * motionPosNoise(noise_generator);
    double turnRad = update->linear.z * motionPosNoise(noise_generator);
    double rotation = update->angular.z * motionHeadingNoise(noise_generator);
    
    if (update->linear.x != 0)
    {
        x += forward * cos(heading);
        y += forward * sin(heading);
    }
    else if (rotation > 0) {
        double x_centre = x - turnRad * sin(heading);
        double y_centre = y + turnRad * cos(heading);
        x = x_centre + turnRad * sin(heading + rotation);
        y = y_centre - turnRad * cos(heading + rotation);
        heading += rotation;
    } else {
        rotation = -rotation;
        double x_centre = x + turnRad * sin(heading);
        double y_centre = y - turnRad * cos(heading);
        x = x_centre - turnRad * sin(heading - rotation);
        y = y_centre + turnRad * cos(heading - rotation);
        heading -= rotation;
    }


}

double CParticle::ComputeParticleProbability(sensor_msgs::LaserScan::ConstPtr&
                                                        scan, CMap* pMap) {
//#if 0

    geometry_msgs::Twist location;
    location.linear.x = x;
    location.linear.y = y;
    location.angular.z = heading;
    sensor_msgs::LaserScan simScan;

    auto start = std::chrono::high_resolution_clock::now();

    pMap->SimulateScanFromPosition(simScan, location);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "simScan time: " << elapsed.count() << " s\n";

start = std::chrono::high_resolution_clock::now();
    double lowestErr = std::numeric_limits<double>::infinity();
    for (int orientation = -90; orientation < 90; orientation++)
    {
        double sumErr = 0;

        for (int i = 0; i < 360; i+= 20)
        {
            double measured = scan->ranges[i];
            double simulated = simScan.ranges[(i+orientation)%360];
            
            if (measured > 8.0)
            {
                measured = 8.0;
            }
            if (simulated > 8.0)
            {
                simulated = 8.0;
            }
            sumErr += (measured - simulated)*(measured - simulated);
            if (sumErr > lowestErr)
            {
                break;
            }

        }
        if (sumErr < lowestErr)
        {
            lowestErr = sumErr;
        }

    }
    finish = std::chrono::high_resolution_clock::now();
    elapsed = finish - start;
    std::cout << "simScan time: " << elapsed.count() << " s\n";
    //if (lowestErr < 7)
      //  std::cout << "Particle x: " << x << "  y: " << y << "  heading: " << heading << "  err: " << (1.0/sqrt(lowestErr)) << "\n";
    return 1.0/sqrt(lowestErr);
//#endif







}

/*****************************************************************/
/* PARTICLE FILTER */
/*****************************************************************/


CParticleFilter::CParticleFilter(CMap* pMap, int numParticles)
: m_bFreshUpdates(false),
  m_pMap(pMap)
{
    noise_generator = std::default_random_engine(time(0));
    randomIndex = std::uniform_int_distribution<int>(0,numParticles);
    double xmin, xmax, ymin, ymax;
    m_pMap->GetExtremes(xmin, xmax, ymin, ymax);

    xUniformDistribution = std::uniform_real_distribution<double>(xmin, xmax);
    yUniformDistribution = std::uniform_real_distribution<double>(ymin, ymax);

    m_pParticleList = new std::vector<CParticle>(numParticles);

}

CParticleFilter::~CParticleFilter() {
    delete m_pParticleList;
    m_pParticleList = nullptr;
}


void CParticleFilter::ProcessMotionUpdate(geometry_msgs::Twist::ConstPtr& update)
{
    //Update the particles
    std::unique_lock<std::mutex> particleLock(m_mutexParticles);

    for (auto &particle : (*m_pParticleList))
    {
        particle.MotionUpdate(update);
    }
    particleLock.unlock();
    std::unique_lock<std::mutex> freshLock(m_mutexFreshUpdates);
    m_bFreshUpdates = true;
    freshLock.unlock();
}


void CParticleFilter::ProcessScanUpdate(sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::unique_lock<std::mutex> freshLock(m_mutexFreshUpdates);
    if (!m_bFreshUpdates)
    {
        return;
    }
    freshLock.unlock();
    std::cout << "processing scan\n";

    //std::vector<double> probabilityList(m_pParticleList->size());
    //calculate probability of each particle
    std::vector<double> probabilityList = m_pMap->GetScanMatchMap()->
                            AssessParticleSet(scan, m_pParticleList);
    for (int i = 0; i < probabilityList.size(); i++)
    {
        probabilityList[i] = sqrt(probabilityList[i]);
    }
/*
    int particleNum = 0;
    for (auto &particle: (*m_pParticleList)) {
        probabilityList[particleNum] = particle.ComputeParticleProbability(scan, m_pMap);
        particleNum++;
    }
*/
    ResampleParticles(probabilityList);
    //re-sample particles


    freshLock.lock();
    m_bFreshUpdates = false;
    freshLock.unlock();
    std::unique_lock<std::mutex> lock(m_mutexParticles);
    for (auto& particle : *m_pParticleList)
    {
        std::cout << "Particle:  "<<particle.x << ", " << particle.y << std::endl;
    }

    std::cout << "done processing scan\n";

}


void CParticleFilter::SetMap(CMap* pMap) {
    m_pMap = pMap;
}

static double maxWeight(std::vector<double>& weightings) {
    double max = 0.0;
    for (auto &weight : weightings) {
        if (weight > max) {
            max = weight;
        }
    }
    return max;
}

void CParticleFilter::ResampleParticles(std::vector<double>& vWeightings) {
    std::vector<CParticle>* pNewParticleList;
    pNewParticleList = new std::vector<CParticle>(0);
    pNewParticleList->reserve(m_pParticleList->size());

    double maxweight = maxWeight(vWeightings) * 2.0;
    resamplerDistribution = std::uniform_real_distribution<double>(0.0, maxweight);
    double beta = 0.0;
    int index = randomIndex(noise_generator);
    for (int i = 0; i < m_pParticleList->size(); i++) {
        beta += resamplerDistribution(noise_generator);
        while (beta > vWeightings[index]) {
            beta -= vWeightings[index];
            index++;
            index %= m_pParticleList->size();
        }
        pNewParticleList->push_back((*m_pParticleList)[index]);
        //std::cout << "Picked particle " << (*pNewParticleList)[i].x << ", " << (*pNewParticleList)[i].y << ", weight " << vWeightings[index] << "\n"; 
    }
    delete m_pParticleList;
    m_pParticleList = pNewParticleList;
}

void CParticleFilter::NormalizeProbabilities(std::vector<double>& vProbabilities) {

}


void CParticleFilter::PublishParticles(ros::Publisher& pub)
{
    PointCloud particleMsg;

    particleMsg.header.frame_id = "world_frame";
    //particleMsg.header.stamp = ros::Time::now();

    std::unique_lock<std::mutex> lock(m_mutexParticles);
    for (auto& particle : *m_pParticleList)
    {
        Point p;
        p.x = particle.x;
        p.y = particle.y;
#ifdef POINT3D
        p.z = 0;
#endif
        particleMsg.push_back(p);
        //std::cout << "Particle:  "<<p.x << ", " << p.y << std::endl;
    }

    lock.unlock();

    pub.publish(particleMsg);

}