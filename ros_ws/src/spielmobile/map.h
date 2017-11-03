#ifndef MAP_H
#define MAP_H


#include <vector>
#include <string>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;


class CMap {
public:
    CMap(std::string file);
    ~CMap();
    bool IsSpaceOccupied(double x, double y);

    void SimulateScanFromPosition(
    				sensor_msgs::LaserScan& scan,
    				geometry_msgs::Twist& pos);
    void PublishMap(ros::Publisher& publisher);

private:
    double m_width;
    double m_height;
    PointCloud::Ptr m_pPointCloud;
    pcl::KdTreeFLANN<Point> m_kdTree;
};


#endif //MAP_H

