#ifndef SLAM_H
#define SLAM_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <thread>
#include <mutex>

class CMap;
class CScanMatchMap;

class CSlam
{
public:
    CSlam();
    void MotionUpdate(geometry_msgs::Twist::ConstPtr& update);
    void ProcessScan(sensor_msgs::LaserScan::ConstPtr& scan);

    void Publish(ros::Publisher& mapPub, ros::Publisher& posPub);

private:
    geometry_msgs::Twist  m_curPos;
    CScanMatchMap* m_pCurSubmap;
    CMap* m_pMap;
    std::mutex m_mutexPos;
    bool m_bFirstScan;

    bool m_bFreshUpdates;
    std::mutex m_mutexFreshUpdates;
};

#endif //SLAM_H