#include <ros/ros.h>

//C++ includes
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <stdint.h>
//#include <pthread.h>
#include <thread>
#include <mutex>

//ROS includes
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
// this is a test
using namespace std;

#define MAX_QUEUE_SIZE 20

sensor_msgs::LaserScan scan360;

class Particle {
	private:
		double x_pos;
		double y_pos;
		double heading;

	public:
		Particle() {

		}

		void motion_update(const geometry_msgs::Twist::ConstPtr& update) {
			
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



/******************************************************************************/
/* GLOBAL VARIABLES */
/******************************************************************************/
Update_Queue motion_update_queue;






/******************************************************************************/
/* MAIN FUNCTIONS */
/******************************************************************************/

/* Callback when a lidar scan has completed
 *
 * @param scan - reference to the completed scan
 * 
 * functionality - replace the global most_recent_scan
 *					so that the partical filter thread can process it when ready
*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	//Do something

}

/* Callback when a motion update has been received from the ARM
 *
 * @param update - twist message containing the motion update
 * 
 * functionality - push the update to the end of the update queue so that the 
 *					motion update thread can update all the particles in the correct motion order
*/
void motionCallback(const geometry_msgs::Twist::ConstPtr& update) {
	//Do something
	motion_update_queue.enqueue(update);
}


void *motion_updater(void* arg) {


	while(1) {
		geometry_msgs::Twist update;
		if (motion_update_queue.dequeue(&update) != 0) {
			continue;
		}

		
	}

}

void *particle_filter(void *arg) {


}


void initialize_map(std::string mapfile) {

}

int main(int argc, char **argv) {
	//Init ros
	ros::init(argc,argv, "Lidar_snapshotter");
	ros::NodeHandle nh;



	ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1,scanCallback);
	ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Twist>("/scan", 20,motionCallback);


	

	ros::spin();
	return 0;

}
