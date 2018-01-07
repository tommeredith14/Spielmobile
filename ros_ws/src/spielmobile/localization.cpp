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

//Other includes
#include "map.h"
#include "ParticleFilter.h"
#include <chrono>
using namespace std;

#define MAX_QUEUE_SIZE 20

sensor_msgs::LaserScan scan360;
/*
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
*/
/*
class Update_Queue {
    private:
        class Update_Node {
            friend Update_Queue;
            private:
                geometry_msgs::Twist update;
                Update_Node *next;

            public:
                Update_Node(const geometry_msgs::Twist::ConstPtr& new_update) {
                    update = *new_update;
                    next = nullptr;
                }
        };
        int m_size;
        Update_Node *m_head;
        Update_Node *m_tail;
        //pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
        std::mutex m_queue_mutex;

    public:
        Update_Queue()
          : m_head(nullptr),
            m_tail(nullptr),
            m_size(0)
        {
        }

        int dequeue(geometry_msgs::Twist *update_twist) {
            //pthread_mutex_lock(&queue_mutex);
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            if (m_size == 0) {
                return -1;
            }

            Update_Node *retnode = m_head;
            *update_twist = retnode->update;
            if (size == 1) {
                m_head = nullptr;
                m_tail = nullptr;
                m_size--;
            } else {
                m_head = retnode->next;
                m_size--;
            }
            delete retnode;
            lock.unlock();
            return 0;
        }

        void enqueue(const geometry_msgs::Twist::ConstPtr& update) {
            Update_Node *new_node = new Update_Node(update);
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            if (m_head == nullptr) {
                m_head = new_node;
                m_tail = new_node;
            } else {
                m_tail->next = new_node;
                m_tail = new_node;

            }
            m_size++;
            if (m_size > MAX_QUEUE_SIZE) {
                cout << "MAXIMUM QUEUE SIZE REACHED - MOTION UPDATES NOT KEEPING UP\n";
            }
            lock.unlock();
        }

};

*/

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

//std::mutex mutexParticles;
//std::vector<CParticle> ParticleList;
CParticleFilter* pParticleFilter;

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
}

ros::Publisher particlePub;

void MotionUpdater() {


    while(ros::ok()) {
        geometry_msgs::Twist::ConstPtr update;
        std::unique_lock<std::mutex> updateLock(mutexMotionQueue);
        condvarMotionUpdate.wait(updateLock,
            []{return (motionUpdateQueue.size() > 0);});

        update = motionUpdateQueue.front();
        motionUpdateQueue.pop();
        updateLock.unlock();

        pParticleFilter->ProcessMotionUpdate(update);

        pParticleFilter->PublishParticles(particlePub);
    }

}



void ParticleFilter() {
    if (pMap == nullptr) {
        return;
    }

    while (ros::ok())
    {
        sensor_msgs::LaserScan::ConstPtr pScan;
        std::unique_lock<std::mutex> scanLock(mutexLastScan);
        condvarScanReady.wait(scanLock,
            []{return bScanReady;});
        bScanReady = false;
        //std::cout << "got a scan\n";
        pScan = lastScan;
        scanLock.unlock();

        pParticleFilter->ProcessScanUpdate(pScan);

        pParticleFilter->PublishParticles(particlePub);

    }

}


void initialize_map(std::string mapfile) {

}

int main(int argc, char **argv) {
    //Init ros
    ros::init(argc,argv, "Lidar_snapshotter");
    ros::NodeHandle nh;

    pMap = new CMap(argv[1]);
    pParticleFilter = new CParticleFilter(pMap);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1,scanCallback);
    ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Twist>("spielmobile/motion_updates", 20,motionCallback);
    ros::Publisher mapPub = nh.advertise<PointCloud>("spielmobile/current_map",1);
    ros::Publisher occupancyPub = nh.advertise<nav_msgs::OccupancyGrid>("spielmobile_occupancy",1);
    ros::Publisher occupancyPubLowRes = nh.advertise<nav_msgs::OccupancyGrid>("spielmobile_occupancy_lowres",1);
    particlePub = nh.advertise<PointCloud>("spielmobile_particles", 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    pMap->PublishMap(mapPub);
    pMap->PublishOccupancyMap(occupancyPub);
    pMap->PublishLowResOccupancyMap(occupancyPubLowRes );

    std::thread motionThread(MotionUpdater);
    std::thread patricleThread(ParticleFilter);


    ros::spin();
    std::terminate();
    motionThread.join();
    patricleThread.join();
    return 0;

}
