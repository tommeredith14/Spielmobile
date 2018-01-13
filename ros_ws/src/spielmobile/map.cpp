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
#ifdef POINT3D
                point.z = 0;
#endif
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


        m_pScanMatchMap = new CScanMatchMap(30,30,0.03,-5,-5);
        m_pScanMatchMap->SetMapPointCloud(m_pPointCloud);
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

double CMap::SqrDistanceBetweenPoints(const Point& first, const Point& second)
{
    return ((first.x - second.x)*(first.x - second.x) +
             (first.y - second.y)*(first.y - second.y));
}

double CMap::DistanceToWall(double x, double y, double heading)
{
    Point startPoint;
    startPoint.x = x;
    startPoint.y = y;
#ifdef POINT3D
    startPoint.z = 0;
#endif
    Point curPoint = startPoint;

    std::vector<float> vectNearestSqrDist(1);
    std::vector<int> indices(1);

    vectNearestSqrDist[0] = 10000;
    while ((vectNearestSqrDist[0]) > NEAR_SQR_DIST_THRESH)
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
        if (SqrDistanceBetweenPoints(startPoint, curPoint) > 64.0)
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

void CMap::PublishOccupancyMap(ros::Publisher& pub)
{
    m_pScanMatchMap->Publish(pub);
}
void CMap::PublishLowResOccupancyMap(ros::Publisher& pub)
{
    m_pScanMatchMap->PublishLowRes(pub);
}

CScanMatchMap::CScanMatchMap(float width, float height, float resolution, float xMin, float yMin)
{
    int xPoints = width/resolution;
    int yPoints = height/resolution;
    m_xMin = xMin;
    m_yMin = yMin;
    m_resolution = resolution;
    m_width = width;
    m_height = height;
    m_scanIter = 0;
    for (int x = 0; x < xPoints; x++)
    {
        std::vector<ScanMatchPoint> vectCol(yPoints);
        for (int y = 0; y < yPoints; y++)
        {
            vectCol[y].x = xMin + x*resolution;
            vectCol[y].y = yMin + y*resolution;
            vectCol[y].prob = 0.1; //TODO
            vectCol[y].bObserved = false;
            vectCol[y].observedInIter = -1;
            //TODO observed
        }
        this->m_mapPoints.push_back(vectCol);
    }

}

ScanMatchPoint* CScanMatchMap::Point(float x, float y, int* pMapX, int* pMapY)
{
    int mapX = (int)(((x - m_xMin)/m_resolution) + 0.5);
    int mapY = (int)(((y - m_yMin)/m_resolution) + 0.5);

    if (mapX < 0 || mapY < 0 || mapX >= m_mapPoints.size() || mapY >= m_mapPoints[0].size())
    {
        return nullptr;
    }

    if (pMapX) *pMapX = mapX;
    if (pMapY) *pMapY = mapY;

    return &m_mapPoints[mapX][mapY];

}

ScanMatchPoint* CScanMatchMap::LowResPoint(float x, float y, int* pMapX, int* pMapY)
{
    int mapX = (int)(((x - m_xMin)/m_lowResolution) + 0.5);
    int mapY = (int)(((y - m_yMin)/m_lowResolution) + 0.5);

    if (mapX < 0 || mapY < 0 || mapX >= m_lowResMapPoints.size() || mapY >= m_lowResMapPoints[0].size())
    {
        return nullptr;
    }

    if (pMapX) *pMapX = mapX;
    if (pMapY) *pMapY = mapY;

    return &m_lowResMapPoints[mapX][mapY];

}
#define GAUSS_RADIUS 0.5
#define GAUSS_STD_DEV 0.25
const double normWeighter = P_HIT;
const double normExpDiviser = 2*(GAUSS_STD_DEV*GAUSS_STD_DEV);

void CScanMatchMap::SetMapPointCloud(PointCloud::Ptr pPointCloud)
{
    for (auto& obsPoint : pPointCloud->points)
    {
        int mapX = -1;
        int mapY = -1;
        ScanMatchPoint* pMapPoint = this->Point(obsPoint.x, obsPoint.y, &mapX, &mapY);

        pMapPoint->prob = P_HIT;

        //Compute gaussian circle around point

        for (int nearX = mapX - GAUSS_RADIUS/m_resolution; 
                nearX < mapX + GAUSS_RADIUS/m_resolution; nearX++)
        {
            if (nearX < 0 || nearX >= m_mapPoints.size()) continue;
            for (int nearY = mapY - GAUSS_RADIUS/m_resolution;
                nearY < mapY + GAUSS_RADIUS/m_resolution; nearY++)
            {
                if (nearY < 0 || nearY >= m_mapPoints[nearX].size()) continue;

                ScanMatchPoint* pNearPoint = &m_mapPoints[nearX][nearY];
                float sqrDist = (obsPoint.x - pNearPoint->x)*(obsPoint.x - pNearPoint->x)
                    +   (obsPoint.y - pNearPoint->y)*(obsPoint.y - pNearPoint->y);
                
                float tempProb = normWeighter * exp(-sqrDist/normExpDiviser);
                if (tempProb > pNearPoint->prob)
                {
                    pNearPoint->prob = tempProb;
                }
            }

        }


    }
    GenerateLowResMap();

}

void CScanMatchMap::GenerateLowResMap()
{
    m_lowResolution = m_resolution*10;
    int lowResXSize = (int)(m_mapPoints.size()/10.0 + 0.5);
    int lowResYSize = (int)(m_mapPoints[0].size()/10.0 + 0.5);

    m_lowResMapPoints.resize(lowResYSize);
    int colNum = 0;
    for (auto& col : m_lowResMapPoints)
    {
        col.resize(lowResYSize);
        int rowNum = 0;
        for (auto& lowResPoint : col)
        {
            lowResPoint.x = m_xMin + colNum * m_lowResolution + m_lowResolution/2;
            lowResPoint.y = m_yMin + rowNum * m_lowResolution + m_lowResolution/2;
            lowResPoint.prob = 0;
            for (int i = colNum*10; i < (colNum+1)*10; i++)
            {
                if (i >= m_mapPoints.size()) continue;
                for (int j = rowNum*10; j < (rowNum+1)*10; j++)
                {   
                    if (j >= m_mapPoints[i].size()) continue;

                    if (m_mapPoints[i][j].prob > lowResPoint.prob)
                    {
                        lowResPoint.prob = m_mapPoints[i][j].prob;
                    }
                }
            }
            rowNum++;

        }
        colNum++;
    }

}
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_datatypes.h>
void CScanMatchMap::Publish(ros::Publisher& pub)
{
    nav_msgs::OccupancyGrid gridMsg;
    gridMsg.header.frame_id = "world_frame";
    gridMsg.header.stamp = ros::Time::now();
    gridMsg.info.resolution = m_resolution;
    gridMsg.info.width = m_mapPoints.size();
    gridMsg.info.height = m_mapPoints[0].size();
    gridMsg.info.origin.position.x = m_xMin - m_resolution/2;
    gridMsg.info.origin.position.y = m_yMin - m_resolution/2;
    tf::Quaternion orient = tf::createQuaternionFromYaw(0);
    tf::quaternionTFToMsg(orient, gridMsg.info.origin.orientation);

    for ( int j = 0; j < m_mapPoints[0].size(); j++)
    {
        for (int i = 0; i < m_mapPoints.size(); i++)
        {
            float p = m_mapPoints[i][j].prob;
            //convert from [0,1] to [0,100]
            int8_t intP = p*100;
            if (!m_mapPoints[i][j].bObserved && m_mapPoints[i][j].observedInIter == -1)
            {
                intP = 0;
            }
            if (intP < 0) intP = 0;
            if (intP > 100) intP = 100;
            gridMsg.data.push_back(intP);
        }
    }
    pub.publish(gridMsg);

}

void CScanMatchMap::PublishLowRes(ros::Publisher& pub)
{
    nav_msgs::OccupancyGrid gridMsg;
    gridMsg.header.frame_id = "world_frame";
    gridMsg.header.stamp = ros::Time::now();
    gridMsg.info.resolution = m_lowResolution;
    gridMsg.info.width = m_lowResMapPoints.size();
    gridMsg.info.height = m_lowResMapPoints[0].size();
    gridMsg.info.origin.position.x = m_xMin;// - m_lowResolution/2;
    gridMsg.info.origin.position.y = m_yMin;// - m_lowResolution/2;

    for ( int j = 0; j < m_lowResMapPoints[0].size(); j++)
    {
        for (int i = 0; i < m_lowResMapPoints.size(); i++)
        {
            float p = m_lowResMapPoints[i][j].prob;
            //convert from [0,1] to [0,100]
            int8_t intP = p*100;
            if (intP < 0) intP = 0;
            if (intP > 100) intP = 100;
            gridMsg.data.push_back(intP);
        }
    }
    pub.publish(gridMsg);

}

#include "ParticleFilter.h"
std::vector<double> CScanMatchMap::AssessParticleSet(sensor_msgs::LaserScan::ConstPtr& scan, 
                            std::vector<CParticle>* pParticleVector)
{
    std::vector<double> results(pParticleVector->size());

    for (int headingDeg = 0;//searchScope.headingMin;
         headingDeg < 360;//searchScope.headingMax;
         headingDeg++)
    {
        //compute points as if around 0,0
        std::vector<::Point> vectScanPoints(360);
        for (int degree = 0; degree < 360; degree++)
        {
            vectScanPoints[degree].x = scan->ranges[degree] * cos((headingDeg + degree) * M_PI/180);
            vectScanPoints[degree].y = scan->ranges[degree] * sin((headingDeg + degree) * M_PI/180);
        }
        int particleNum = 0;
        for (auto& particle : *pParticleVector)
        {
            double score = AssessPoint(particle.x,particle.y, vectScanPoints);
            if (score > results[particleNum])
            {
                results[particleNum] = score;
            }

            particleNum++;
        }
    }
    for (int i = 0; i < pParticleVector->size(); i++)
    {
        std::cout << "Particle: " << (*pParticleVector)[i].x <<", " << (*pParticleVector)[i].y << "  Prob: " << results[i] << "\n";

    }

    return results;
}

double CScanMatchMap::AssessPoint(double x, double y, const std::vector<::Point>& vectScanPoints)
{
    double score = 1.0;
    for (auto& scanPoint : vectScanPoints)
    {
        ScanMatchPoint* matchPoint = this->Point(scanPoint.x + x, scanPoint.y + y, nullptr, nullptr);
        score *= matchPoint->prob; 
    }
    return score;
}

void CScanMatchMap::SetUpForSlam()
{
    //TODO (blank for now)
}

static double odds(double p)
{
    return p/(1-p);
}
static double invOdds(double odds)
{
    return odds/(1+odds);
}

static const double ODDS_PHIT = odds(P_HIT);
static const double ODDS_PMISS = odds(P_MISS);

static double ProbUpdate(double old, bool hit)
{
    double newProb = invOdds(odds(old)*((hit)?ODDS_PHIT:ODDS_PMISS));
    if (newProb < 0)
    {
        newProb = 0;
    }
    if (newProb > 1)
    {
        newProb = 1;
    }
    return newProb;
}


void CScanMatchMap::GaussianAboutPoint(double x, double y)
{
        int mapX = -1;
        int mapY = -1;
        ScanMatchPoint* pMapPoint = this->Point(x, y, &mapX, &mapY);

        pMapPoint->prob = P_HIT;

        //Compute gaussian circle around point

        for (int nearX = mapX - GAUSS_RADIUS/m_resolution; 
                nearX < mapX + GAUSS_RADIUS/m_resolution; nearX++)
        {
            if (nearX < 0 || nearX >= m_mapPoints.size()) continue;
            for (int nearY = mapY - GAUSS_RADIUS/m_resolution;
                nearY < mapY + GAUSS_RADIUS/m_resolution; nearY++)
            {
                if (nearY < 0 || nearY >= m_mapPoints[nearX].size()) continue;

                ScanMatchPoint* pNearPoint = &m_mapPoints[nearX][nearY];
                if (pNearPoint->bObserved) continue;
                float sqrDist = (x - pNearPoint->x)*(x - pNearPoint->x)
                    +   (y - pNearPoint->y)*(y - pNearPoint->y);
                
                float tempProb = normWeighter * exp(-sqrDist/normExpDiviser);
                if (tempProb > pNearPoint->prob)
                {
                    pNearPoint->prob = tempProb;
                    pNearPoint->observedInIter = m_scanIter;
                    ScanMatchPoint* pLowResPoint = LowResPoint(pNearPoint->x, pNearPoint->y);
                    if (pLowResPoint)
                    {
                        pLowResPoint->observedInIter = m_scanIter;
                    }
                }
            }

        }
}

void CScanMatchMap::InsertScan(sensor_msgs::LaserScan::ConstPtr& scan, 
            geometry_msgs::Twist& pos)
{
    std::vector<ScanMatchPoint*> vectHits(0);
    std::vector<ScanMatchPoint*> vectMisses(0);
    // for each ray
    for (int deg = 0; deg < 360; deg++)
    {
        // insert the endpoint as a hit. If its new, spread it as a gaussian, else
        // just do the calculation
        if (scan->ranges[deg] != std::numeric_limits<float>::infinity())
        {
            
            double endx = pos.linear.x + scan->ranges[deg]* cos((deg)*M_PI/180 + pos.angular.z);
            double endy = pos.linear.y + scan->ranges[deg]* sin((deg)*M_PI/180 + pos.angular.z);

            ScanMatchPoint* pHitPoint = this->Point(endx, endy, nullptr, nullptr);
            if (!pHitPoint->bObserved /*|| pHitPoint->observedInIter < m_scanIter*/)
            {
                //spread gaussian around point
                pHitPoint->bObserved = true;
                pHitPoint->observedInIter = m_scanIter;
                GaussianAboutPoint(endx,endy);
            }
            else
            {
                //update probability if not already sees iterationn
                if (pHitPoint->observedInIter < m_scanIter)
                {
                    pHitPoint->prob = ProbUpdate(pHitPoint->prob, true);
                    pHitPoint->observedInIter = m_scanIter;
                }
            }
            ScanMatchPoint* pLowResPoint = LowResPoint(pHitPoint->x,pHitPoint->y);
            if (pLowResPoint)
            {
                pLowResPoint->observedInIter = m_scanIter;
            }
        }
        

        // go through missed grids, and update if not touched yet
        // based on http://www.idav.ucdavis.edu/education/GraphicsNotes/Bresenhams-Algorithm.pdf page 9
        double dx = cos(deg*M_PI/180 + pos.angular.z);//(endx - pos.linear.x)/scan->ranges[deg];
        double dy = sin(deg*M_PI/180 + pos.angular.z);//(endy - pos.linear.y)/scan->ranges[deg];
        double dist;
        if (scan->ranges[deg] != std::numeric_limits<float>::infinity())
        {
            dist = scan->ranges[deg];
        }
        else
        {
            dist = 8;
        }
        if (fabs(dx) > fabs(dy))
        {
            //x is driving axis
            double slope = dy/dx;
            double err = slope - 1;
            double y = pos.linear.y;
            double endx = pos.linear.x + dist*dx;
            int direction = (dx > 0)?1:-1;
            double yStep = m_resolution*slope*direction;
            for (double x = pos.linear.x; (direction == 1 && x < endx) ||
                            (direction == -1 && x > endx);
                             x+= m_resolution*direction)
            {
                ScanMatchPoint* pPoint = this->Point(x,y,nullptr,nullptr);
                if (pPoint)
                {
                    if (!pPoint->bObserved && pPoint->observedInIter < m_scanIter)
                    {
                        pPoint->prob = P_MISS;
                        pPoint->bObserved = true;
                        pPoint->observedInIter = m_scanIter;
                        ScanMatchPoint* pLowResPoint = LowResPoint(pPoint->x,pPoint->y);
                        if (pLowResPoint)
                        {
                            pLowResPoint->observedInIter = m_scanIter;
                        }
                    }
                    else if (pPoint->observedInIter < m_scanIter)
                    {
                        pPoint->prob = ProbUpdate(pPoint->prob, false);
                        pPoint->observedInIter = m_scanIter;
                        ScanMatchPoint* pLowResPoint = LowResPoint(pPoint->x,pPoint->y);
                        if (pLowResPoint)
                        {
                            pLowResPoint->observedInIter = m_scanIter;
                        }
                    }
                }
                y += yStep;
            }
        }
        else
        {
            //y is driving axis
            double slope = dx/dy;
            double err = slope - 1;
            double x = pos.linear.x;
            double endy = pos.linear.y + dy*dist;
            int direction = (dy > 0)?1:-1;
            double xStep = m_resolution*slope*direction;
            for (double y = pos.linear.y; 
                (direction == 1 && y < endy) ||
                    (direction == -1 && y > endy);
                y+= m_resolution*direction)
            {
                ScanMatchPoint* pPoint = this->Point(x,y,nullptr,nullptr);
                if (pPoint)
                {
                    if (!pPoint->bObserved && pPoint->observedInIter < m_scanIter)
                    {
                        pPoint->prob = P_MISS;
                        pPoint->bObserved = true;
                        pPoint->observedInIter = m_scanIter;
                        ScanMatchPoint* pLowResPoint = LowResPoint(pPoint->x,pPoint->y);
                        if (pLowResPoint)
                        {
                            pLowResPoint->observedInIter = m_scanIter;
                        }
                    }
                    else if (pPoint->observedInIter < m_scanIter)
                    {
                        pPoint->prob = ProbUpdate(pPoint->prob, false);
                        pPoint->observedInIter = m_scanIter;
                        ScanMatchPoint* pLowResPoint = LowResPoint(pPoint->x,pPoint->y);
                        if (pLowResPoint)
                        {
                            pLowResPoint->observedInIter = m_scanIter;
                        }
                    }
                }
                x += xStep;
            }
        }


    }

    //update the low-rez map as needed
    int colNum = 0;
    for (auto& row : m_lowResMapPoints)
    {
        int rowNum = 0;
        for (auto& lowResPoint : row)
        {
            if (lowResPoint.observedInIter == m_scanIter)
            {
                for (int i = colNum*10; i < (colNum+1)*10; i++)
                {
                    if (i >= m_mapPoints.size()) continue;
                    for (int j = rowNum*10; j < (rowNum+1)*10; j++)
                    {   
                        if (j >= m_mapPoints[i].size()) continue;

                        if (m_mapPoints[i][j].prob > lowResPoint.prob)
                        {
                            lowResPoint.prob = m_mapPoints[i][j].prob;
                        }
                    }
                }
            }
            rowNum++;
        }
        colNum++;
    }
    m_scanIter++;
}


ScanMatchPoint CScanMatchMap::AssessScan(sensor_msgs::LaserScan::ConstPtr& scan,
                                         const SearchScope& searchScope)
{
   /* for (int headingDeg = searchScope.headingMin;
         headingDeg < searchScope.headingMax;
         headingDeg++)
    {
        //compute points as if around 0,0
        std::vector<Point> vectScanPoints(360);
        for (int degree = 0; degree < 360; degree++)
        {
            vectScanPoints[degree].x = scan->data[degree] * cos(headingDeg + degree);
            vectScanPoints[degree].y = scan->data[degree] * sin(headingDeg + scan);
        }
        for (auto& col : m_mapPoints)
        {
            for (auto& mapPoint : col)
            {
                
            }
        }
    }*/
    return ScanMatchPoint();
}

