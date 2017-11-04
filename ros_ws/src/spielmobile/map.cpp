#include "map.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>
#include <limits>

CMap::CMap(std::string filename) {
    m_pPointCloud = PointCloud::Ptr(new PointCloud());

    if (filename.find_last_of(".vmap") != std::string::npos) {
        std::vector<Point> points;
        std::ifstream file(filename);
        if (!file.is_open())
        {
            return;
        }
        std::string input;
        while (file >> input)
        {
            //0.0,0.0-0.0,0.0
            int firstComma = input.find_first_of(',');
            int secondComma = input.find_last_of(',');
            int hyphen = input.find_first_of('-');
            if (firstComma == std::string::npos ||
                secondComma == std::string::npos ||
                hyphen == std::string::npos)
            {
                continue;
            }

            std::string x1 = input.substr(0,firstComma);
            std::string y1 = input.substr(firstComma+1, hyphen - firstComma-1);
            std::string x2 = input.substr(hyphen+1, secondComma - hyphen-1);
            std::string y2 = input.substr(secondComma+1);

            double startx = stod(x1);
            double starty = stod(y1);
            double endx = stod(x2);
            double endy = stod(y2);

            std::cout << startx <<" "<<starty<<" "<<endx<<" "<<endy<<"\n";
            double dx = endx - startx;
            double dy = endy - starty;
            double length = sqrt(dx*dx + dy*dy);
            //unit vector
            dx /= length;
            dy /= length;

            for (double travelled = 0; travelled < length; travelled+= 0.01)
            {
                Point point;
                point.x = startx + travelled*dx;
                point.y = starty + travelled*dy;
                point.z = 0;
                m_pPointCloud->push_back(point);
            }

        }



        //pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

          //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXY> transformed_cloud_color_handler (m_pPointCloud, 230, 20, 20); // Red
          //viewer.addPointCloud (m_pPointCloud, "cloud");//transformed_cloud_color_handler, "transformed_cloud");

          //viewer.addCoordinateSystem (1.0, "cloud", 0);
          //viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
          //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
          //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
          //viewer.setPosition(800, 400); // Setting visualiser window position

          //while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
            //viewer.spinOnce ();
          //}

        m_kdTree.setInputCloud(m_pPointCloud);
    }
}

void CMap::SimulateScanFromPosition(
                sensor_msgs::LaserScan& scan,
               const geometry_msgs::Twist& pos)
{
    double x = pos.linear.x;
    double y = pos.linear.y;
    double heading = pos.angular.z;

    scan.angle_min = 0;
    scan.angle_max = 359*M_PI/180;
    scan.header.frame_id = "world_frame";
    scan.header.stamp = ros::Time::now();
    scan.angle_increment = M_PI/180;
    scan.range_min = 0.0f;
    scan.range_max = 6.0f;

    for (int i = 0; i < 360; i++)
    {
        scan.ranges.push_back(
            DistanceToWall(x,y, heading + scan.angle_increment*i));

    }


}

void CMap::PublishMap(ros::Publisher& publisher)
{
    publisher.publish(*m_pPointCloud);
}

double CMap::DistanceBetweenPoints(const Point& first, const Point& second)
{
    return sqrt((first.x - second.x)*(first.x - second.x) +
             (first.y - second.y)*(first.y - second.y));
}

double CMap::DistanceToWall(double x, double y, double heading)
{
    Point startPoint;
    startPoint.x = x;
    startPoint.y = y;
    startPoint.z = 0;
    Point curPoint = startPoint;

    std::vector<float> vectNearestSqrDist(1);
    std::vector<int> indices(1);

    vectNearestSqrDist[0] = 10000;
    while (sqrt(vectNearestSqrDist[0]) > NEAR_DIST_THRESH)
    {
        if (m_kdTree.nearestKSearch(curPoint,1,indices,vectNearestSqrDist) != 1) {
            //ERROR
            std::cout << "No closest point\n";
        }
        else
        {
            float distToMove = sqrt(vectNearestSqrDist[0]);
            curPoint.x += distToMove*cos(heading);
            curPoint.y += distToMove*sin(heading);

        }
        if (DistanceBetweenPoints(startPoint, curPoint) > 8.0)
        {
            return std::numeric_limits<float>::infinity();
        }

    }


    return DistanceBetweenPoints(startPoint, curPoint);
}

void CMap::GetExtremes(double& xmin, double& xmax, double& ymin, double& ymax)
{
    xmin = ymin = INT_MAX;
    ymax = ymax = -INT_MAX;
    for (auto& point : m_pPointCloud->points)
    {
        if (point.x > xmax)
        {
            xmax = point.x;
        }
        if (point.x < xmin)
        {
            xmin = point.x;
        }

        if (point.y > ymax)
        {
            ymax = point.y;
        }
        if (point.y < ymin)
        {
            ymin = point.y;
        }

    }

}