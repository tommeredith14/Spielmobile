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
#include <nav_msgs/OccupancyGrid.h>

//#define POINT3D

#ifdef POINT3D
typedef pcl::PointXYZ Point;
#else
typedef pcl::PointXY Point;
#endif
typedef pcl::PointCloud<Point> PointCloud;

#define NEAR_DIST_THRESH 0.05
#define NEAR_SQR_DIST_THRESH 0.0025

#define P_HIT 0.7
#define P_MISS 0.3

class SearchScope
{
public:
    float xMin;
    float xMax;
    float yMin;
    float yMax;
    float headingMin;
    float headingMax;
};

class ScanMatchPoint
{
public:
    ScanMatchPoint()
    : prob(P_MISS),
      x(0),
      y(0),
      probOfScan(-1)
    {}
    float prob;
    float x;
    float y;
    float probOfScan;
};

class CParticle;
class CScanMatchMap
{
public:
    CScanMatchMap(float width, float height, float resolution, float xMin, float yMin);
    void SetMapPointCloud(PointCloud::Ptr);
    ScanMatchPoint* Point(float x, float y, int* mapX = nullptr, int* mapY = nullptr);
    void Publish(ros::Publisher& pub);
    void PublishLowRes(ros::Publisher& pub);
    ScanMatchPoint AssessScan(sensor_msgs::LaserScan::ConstPtr& scan,
                               const SearchScope& searchScope);
    std::vector<double> AssessParticleSet(sensor_msgs::LaserScan::ConstPtr& scan, 
                                std::vector<CParticle>* pParticleVector);
    double AssessPoint(double x, double y, const std::vector<::Point>& vectScanPoints);

private:
    void GenerateLowResMap();
    //TODO : assess point given vector of scan points 
    typedef std::vector<ScanMatchPoint> Column;

    std::vector< Column > m_mapPoints;
    std::vector< Column > m_lowResMapPoints;
    float m_width;
    float m_height;
    float m_resolution;
    float m_xMin;
    float m_yMin;

    float m_lowResolution;


};



class CMap {
public:
    CMap(std::string file);
    ~CMap();
    bool IsSpaceOccupied(double x, double y);

    void SimulateScanFromPosition(
    				sensor_msgs::LaserScan& scan,
    				const geometry_msgs::Twist& pos);
    void PublishMap(ros::Publisher& publisher);
    void PublishOccupancyMap(ros::Publisher& publisher);
    void PublishLowResOccupancyMap(ros::Publisher& publisher);

    void GetExtremes(double& xmin, double& xmax, double& ymin, double& ymax);
	double DistanceToWall(double x, double y, double heading);
    CScanMatchMap* GetScanMatchMap() {return m_pScanMatchMap;}
private:

	double DistanceBetweenPoints(const Point& first, const Point& second);
	double SqrDistanceBetweenPoints(const Point& first, const Point& second);
    double m_width;
    double m_height;
    PointCloud::Ptr m_pPointCloud;
    pcl::KdTreeFLANN<Point> m_kdTree;

    //scan matcher map
    CScanMatchMap* m_pScanMatchMap;
};


#endif //MAP_H

