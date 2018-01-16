#include "slam.h"
#include "map.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <mutex>


CSlam::CSlam()
    : m_bFirstScan(true),
      m_pCurSubmap(nullptr),
      m_pMap(nullptr),
      m_bFreshUpdates(true)
{
    m_pCurSubmap = new CScanMatchMap(10,10,0.03,-5,-5);
    m_pCurSubmap->SetUpForSlam();



}

void CSlam::MotionUpdate(geometry_msgs::Twist::ConstPtr& update)
{
    if (m_bFirstScan)
    {
        return;
    }

    std::unique_lock<std::mutex> posLock(m_mutexPos);
    double forward = update->linear.x;
    double turnRad = update->linear.z;
    double rotation = update->angular.z;
    if (update->linear.x != 0)
    {
        m_curPos.linear.x += forward * cos(m_curPos.angular.z);
        m_curPos.linear.y += forward * sin(m_curPos.angular.z);
    }
    else if (rotation > 0) {
        double x_centre = m_curPos.linear.x - turnRad * sin(m_curPos.angular.z);
        double y_centre = m_curPos.linear.y + turnRad * cos(m_curPos.angular.z);
        m_curPos.linear.x = x_centre + turnRad * sin(m_curPos.angular.z + rotation);
        m_curPos.linear.y = y_centre - turnRad * cos(m_curPos.angular.z + rotation);
        m_curPos.angular.z += rotation;
    } else {
        rotation = -rotation;
        double x_centre = m_curPos.linear.x + turnRad * sin(m_curPos.angular.z);
        double y_centre = m_curPos.linear.y - turnRad * cos(m_curPos.angular.z);
        m_curPos.linear.x = x_centre - turnRad * sin(m_curPos.angular.z - rotation);
        m_curPos.linear.y = y_centre + turnRad * cos(m_curPos.angular.z - rotation);
        m_curPos.angular.z -= rotation;
    }

    posLock.unlock();

    std::unique_lock<std::mutex> freshLock(m_mutexFreshUpdates);
    m_bFreshUpdates = true;
    freshLock.unlock();
}

void CSlam::ProcessScan(sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::unique_lock<std::mutex> freshLock(m_mutexFreshUpdates);
    if (!m_bFreshUpdates)
    {
        return;
    }
    freshLock.unlock();
    std::cout << "processing scan\n";
    std::unique_lock<std::mutex> posLock(m_mutexPos);

    if (m_bFirstScan)
    {
        geometry_msgs::Twist pos;
        pos.linear.x = 0;
        pos.linear.y = 0;
        pos.angular.z = 0;
        m_pCurSubmap->InsertScan(scan, pos);
        m_curPos = pos;
        m_bFirstScan = false;
        posLock.unlock();
        freshLock.lock();
        m_bFreshUpdates = false;
        freshLock.unlock();    
        return;
    }

    //Assess the scan and insert it into the map
    SearchScope searchScope;
    searchScope.xMin = m_curPos.linear.x - 1;
    searchScope.yMin = m_curPos.linear.y - 1;
    searchScope.xMax = m_curPos.linear.x + 1;
    searchScope.yMax = m_curPos.linear.y + 1;
    searchScope.headingMin = m_curPos.angular.z * 180/M_PI - 30;
    searchScope.headingMax = m_curPos.angular.z * 180/M_PI + 30;
    ScanMatchPoint bestPoint = m_pCurSubmap->AssessScan(scan,searchScope);

    m_curPos.linear.x = bestPoint.x;
    m_curPos.linear.y = bestPoint.y;
    m_curPos.angular.z = bestPoint.bestHeading;
    m_pCurSubmap->InsertScan(scan, m_curPos);
    posLock.unlock();

    freshLock.lock();
    m_bFreshUpdates = false;
    freshLock.unlock();
    std::cout << "Done scan\n";
}

void CSlam::Publish(ros::Publisher& mapPub, ros::Publisher& posPub)
{
        std::unique_lock<std::mutex> posLock(m_mutexPos);
        geometry_msgs::Twist pos = m_curPos;
        posLock.unlock();   

        m_pCurSubmap->Publish(mapPub);
        
        visualization_msgs::Marker locationMsg;
		locationMsg.header.frame_id = "world_frame";
		locationMsg.header.stamp = ros::Time::now();
		locationMsg.ns = "Spielmobile_SLAM";
		locationMsg.action = visualization_msgs::Marker::ADD;
		locationMsg.type = visualization_msgs::Marker::ARROW;
		locationMsg.id = 0;
		locationMsg.color.g = 1.0f;
		locationMsg.color.b = 1.0f;
		locationMsg.color.a = 1.0;
		locationMsg.pose.position.x = pos.linear.x;
		locationMsg.pose.position.y = pos.linear.y;
		locationMsg.pose.position.z = 0;
		tf::Quaternion orient = tf::createQuaternionFromYaw(pos.angular.z);
		tf::quaternionTFToMsg(orient, locationMsg.pose.orientation);
		locationMsg.scale.x = 0.3;
		locationMsg.scale.y = 0.1;
		locationMsg.scale.z = 0.1;
        posPub.publish(locationMsg);
}
