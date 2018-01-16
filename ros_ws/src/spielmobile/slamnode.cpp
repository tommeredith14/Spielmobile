#include <ros/ros.h>

//C++ includes
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <stdint.h>
//#include <pthread.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <queue>

//ROS includes
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>


//Other includes
#include "map.h"
#include "slam.h"
#include <chrono>
using namespace std;

#define MAX_QUEUE_SIZE 20

sensor_msgs::LaserScan scan360;

/******************************************************************************/
/* GLOBAL VARIABLES */
/******************************************************************************/
//Update_Queue motion_update_queue;
CMap *pMap = nullptr;

std::condition_variable condvarMotionUpdate;
std::mutex mutexMotionQueue;
std::queue<geometry_msgs::Twist::ConstPtr> motionUpdateQueue;

std::condition_variable condvarScanReady;
bool bScanReady = false;
std::mutex mutexLastScan;
sensor_msgs::LaserScan::ConstPtr lastScan;

ros::Publisher slamMapPub;
ros::Publisher posPub;
 
//std::mutex mutexParticles;
//std::vector<CParticle> ParticleList;
//CParticleFilter* pParticleFilter;
CSlam* pSLAM;

void MotionUpdater();
void DoSlam();
/******************************************************************************/
/* MAIN FUNCTIONS */
/******************************************************************************/

/* Callback when a lidar scan has completed
 *
 * @param scan - reference to the completed scan
 * 
 * functionality - replace the global most_recent_scan
 *                    so that the partical filter thread can process it when ready
*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    //Do something
    std::unique_lock<std::mutex> lock(mutexLastScan);
    lastScan = scan;
    bScanReady = true;
    lock.unlock();
    condvarScanReady.notify_all();
    DoSlam();
}

/* Callback when a motion update has been received from the ARM
 *
 * @param update - twist message containing the motion update
 * 
 * functionality - push the update to the end of the update queue so that the 
 *                    motion update thread can update all the particles in the correct motion order
*/
void motionCallback(const geometry_msgs::Twist::ConstPtr& update) {
    //Do something
    std::unique_lock<std::mutex> lock(mutexMotionQueue);
    motionUpdateQueue.push(update);
    lock.unlock();
    condvarMotionUpdate.notify_all();

    if (motionUpdateQueue.size() > MAX_QUEUE_SIZE) {
        std::cout << "The motion queue is larger than its ideal maximum size:  "
            "filter thread is not keeping up\n";
    }
    MotionUpdater();
}

ros::Publisher particlePub;

void MotionUpdater() {


    //while(ros::ok()) {
        geometry_msgs::Twist::ConstPtr update;
        std::unique_lock<std::mutex> updateLock(mutexMotionQueue);
        condvarMotionUpdate.wait(updateLock,
            []{return (motionUpdateQueue.size() > 0);});

        update = motionUpdateQueue.front();
        motionUpdateQueue.pop();
        updateLock.unlock();

        pSLAM->MotionUpdate(update);

        //pSLAM->PublishParticles(particlePub);
        pSLAM->Publish(slamMapPub,posPub);
    //}

}



void DoSlam() {
    //while (ros::ok())
    {
        sensor_msgs::LaserScan::ConstPtr pScan;
        std::unique_lock<std::mutex> scanLock(mutexLastScan);
        condvarScanReady.wait(scanLock,
            []{return bScanReady;});
        bScanReady = false;
        //std::cout << "got a scan\n";
        pScan = lastScan;
        scanLock.unlock();

        pSLAM->ProcessScan(pScan);

        pSLAM->Publish(slamMapPub,posPub);

    }

}


void initialize_map(std::string mapfile) {

}

int main(int argc, char **argv) {
    //Init ros
    ros::init(argc,argv, "Lidar_snapshotter");
    ros::NodeHandle nh;

    //pMap = new CMap(argv[1]);
    pSLAM = new CSlam();

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1,scanCallback);
    ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Twist>("spielmobile/motion_updates", 100,motionCallback);
    //ros::Publisher mapPub = nh.advertise<PointCloud>("spielmobile/current_map",1);
    slamMapPub = nh.advertise<nav_msgs::OccupancyGrid>("spielmobile_map2",1);
    posPub = nh.advertise<visualization_msgs::Marker>("spielmobile_pos",1);
    //ros::Publisher occupancyPubLowRes = nh.advertise<nav_msgs::OccupancyGrid>("spielmobile_occupancy_lowres",1);
    //particlePub = nh.advertise<PointCloud>("spielmobile_particles", 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //pMap->PublishMap(mapPub);
    //pMap->PublishOccupancyMap(occupancyPub);
    //pMap->PublishLowResOccupancyMap(occupancyPubLowRes );

    //std::thread motionThread(MotionUpdater);
    //std::thread scanThread(DoSlam);


    ros::spin();
    std::terminate();
    //motionThread.join();
    //scanThread.join();
    return 0;

}
