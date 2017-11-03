#include "map.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>


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
                geometry_msgs::Twist& pos)
{
    double x = pos.linear.x;
    double y = pos.linear.y;
    double heading = pos.angular.z;





}

void CMap::PublishMap(ros::Publisher& publisher)
{
    publisher.publish(*m_pPointCloud);
}

