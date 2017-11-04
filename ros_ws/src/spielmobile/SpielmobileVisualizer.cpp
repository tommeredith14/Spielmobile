#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <mutex>




class CSpielmobileVilsualizer {
	sensor_msgs::PointCloud2::ConstPtr m_pMapCloud;
	geometry_msgs::Twist::ConstPtr m_pRobotLocation;

	//TODO: Particles

	std::mutex mapMutex;
	std::mutex locationMutex;

	ros::NodeHandle& m_nh;
	std::thread m_spinThread;

	ros::Publisher m_vizPub;
	ros::Subscriber m_locationSub;
	ros::Subscriber m_mapSub;
	ros::Publisher m_mapPub;

public:
	CSpielmobileVilsualizer(ros::NodeHandle& nh)
	: m_nh(nh)
	{
		//m_nh = nh;
		m_spinThread = std::thread(&CSpielmobileVilsualizer::SpinnerThread, this);
		m_vizPub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker",1);
		m_locationSub = m_nh.subscribe("spielmobile/current_location", 1,
					&CSpielmobileVilsualizer::LocationCallback,this);
		m_mapSub = m_nh.subscribe("spielmobile/current_map", 1,
					&CSpielmobileVilsualizer::MapCallback, this);
		m_mapPub = m_nh.advertise<sensor_msgs::PointCloud2>("spielmobile_map", 1);
	}

	~CSpielmobileVilsualizer()
	{
		if (m_spinThread.joinable()) m_spinThread.join();
	}
	void SpinnerThread()
	{
		ros::spin();
	}

	void MapCallback(const sensor_msgs::PointCloud2::ConstPtr& mapCloud) {
		mapMutex.lock();
		m_pMapCloud = mapCloud;
		mapMutex.unlock();
	}

	void LocationCallback(const geometry_msgs::Twist::ConstPtr& location) {
		locationMutex.lock();
		m_pRobotLocation = location;
		locationMutex.unlock();
	}

	void DrawFrame() {
		mapMutex.lock();
		locationMutex.lock();
		if (m_pRobotLocation != nullptr)
		{
			visualization_msgs::Marker locationMsg;
			locationMsg.header.frame_id = "world_frame";
			locationMsg.header.stamp = ros::Time::now();
			locationMsg.ns = "Spielmobile_Visualizer";
			locationMsg.action = visualization_msgs::Marker::ADD;
			locationMsg.type = visualization_msgs::Marker::ARROW;
			locationMsg.id = 0;
			locationMsg.color.g = 1.0f;
			locationMsg.color.a = 1.0;
			locationMsg.pose.position.x = m_pRobotLocation->linear.x;
			locationMsg.pose.position.y = m_pRobotLocation->linear.y;
			locationMsg.pose.position.z = 0;
			tf::Quaternion orient = tf::createQuaternionFromYaw(m_pRobotLocation->angular.z);
			tf::quaternionTFToMsg(orient, locationMsg.pose.orientation);
			locationMsg.scale.x = 0.3;
			locationMsg.scale.y = 0.1;
			locationMsg.scale.z = 0.1;


			m_vizPub.publish(locationMsg);
		}
		if (m_pMapCloud != nullptr)
		{

			sensor_msgs::PointCloud2 mapMsg = *m_pMapCloud;
			mapMsg.header.frame_id = "world_frame";
			mapMsg.header.stamp = ros::Time::now();

			m_mapPub.publish(mapMsg);
		}

		mapMutex.unlock();
		locationMutex.unlock();

	}

};




int main (int argc, char** argv)
{
	ros::init(argc, argv, "Spielmobile_Visualizer");
	ros::NodeHandle nh;
	CSpielmobileVilsualizer SpielmobileVisualizer(nh);

	ros::Rate rate(1);
	while (ros::ok())
	{

		SpielmobileVisualizer.DrawFrame();
		rate.sleep();
	}

}
