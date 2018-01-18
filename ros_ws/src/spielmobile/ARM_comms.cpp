#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <linux/i2c.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <cmath>

using namespace std;

#define CONVERSION_FACTOR 1000.0
#define ZERO_RANGE 0.01
#define WHEELDIST 0.30


int file_i2c;


void generate_motion_update(geometry_msgs::Twist & msg, int right, int left) {
    double right_dist = right / CONVERSION_FACTOR;
    double left_dist = left / CONVERSION_FACTOR;
    if (fabs(right_dist - left_dist) < ZERO_RANGE) {
        msg.angular.z = 0.0;
        msg.linear.x = right_dist/2 + left_dist/2;
        msg.linear.z = 0.0;
        return;
    } else {
        if (left_dist > right_dist) {
            cout << "Right dist " << right_dist << " left dist " <<left_dist << endl;
            double radius = left_dist *WHEELDIST/ (left_dist-right_dist);
            cout << "radius " << radius << endl;
            double theta = left_dist/radius;
            cout << "theta " << theta << endl;
            double mid_radius = radius - WHEELDIST / 2;
            cout << "midrad" << mid_radius << endl;
            msg.linear.x = 0;
            msg.linear.z = mid_radius;
            msg.angular.z = - theta;
            return;
        }
        if (right_dist > left_dist) {
            double radius = right_dist * WHEELDIST / (right_dist - left_dist);
            double theta = right_dist/radius;
            double mid_radius = radius - WHEELDIST / 2;
            msg.linear.x = 0;
            msg.linear.z = mid_radius;
            msg.angular.z = theta;
            return;
        }
    }

}

int rightPower = 0;
int leftPower = 0;


double vMax = 1.0;
double wMax = 1.0;
void MotionCommandCallback(const geometry_msgs::Twist::ConstPtr& cmdVelMsg)
{
    double rightSpeed = 0;
    double leftSpeed = 0;

    rightSpeed = cmdVelMsg->angular.z/wMax;
    leftSpeed = -cmdVelMsg->angular.z/wMax;

    rightSpeed += cmdVelMsg->linear.x/vMax;
    leftSpeed += cmdVelMsg->linear.x/vMax;

    rightPower = round(rightSpeed);
    leftPower = round(leftSpeed);

    if (rightPower > 100)
    {
        rightPower = 100;
    }
    if (rightPower < -100)
    {
        rightPower = -100;
    }
    if (leftPower > 100)
    {
        leftPower = 100;
    }
    if (leftPower < -100)
    {
        leftPower = -100;
    }
}

int main(int argc, char **argv) {
    //Init ros
    ros::init(argc,argv, "ARM_comms");
    ros::NodeHandle nh;

    //Create publisher, set message type
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("spielmobile/motion_updates", 100);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, MotionCommandCallback);
    //Init i2c
    int fd, result;
    char *filename = (char*)"/dev/i2c-1";
    file_i2c = open(filename, O_RDWR);
    if (file_i2c < 0)
    {
        cout << "Init failed\n";
    }



    ros::Rate rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Twist msg;

        //set up for I2C transmission to slave (not sure if this has to be done each time)
        if (ioctl(file_i2c, I2C_SLAVE, 11) < 0) {
            cout << "FAILED ioctl";
        }


        uint8_t to_send[2] = {0,0};

        //Get motor speeds to send
        //TODO: receive this from other nodes

        /*int16_t input;
        cout << "Left wheel: ";
        cin >> input;
        to_send[0]= (uint8_t)input;

        cin.clear();
        cin.ignore(99999, '\n');

        cout << "Right wheel: ";
        cin >> input;
        to_send[1]= (uint8_t)input;*/
        to_send[0] = (uint8_t)leftPower;
        to_send[1] = (uint8_t)rightPower;

        cout << "sending: " << (int)to_send[0] << ", " << (int)to_send[1] << endl;


        uint8_t to_read[6] = {0};
        //Send the I2C message containing motor speeds
        if (write(file_i2c,to_send,2) != 2) {
            cout << "failed write\n";
            goto exit;
        }
        
        //Give the micro a chance to process what it just received
        //TODO: see how short we can push this before the OS lets us down
        usleep(1000);

        if (read(file_i2c, to_read,6) != 6) {
            cout << "failed read";
            goto exit;
        }

        //debugging
        for (int i = 0; i < 6; i++)
            cout << (uint16_t)to_read[i] << endl;

        cin.clear();
        cin.ignore(99999, '\n');

        //Decipher the values
        // 00SF_XXXX XXXX_XXXX XXXX_XXXX
        // -->
        // xxxx_xxxx xxxx_xx00 0011_1111 1222_2222
        int left_dist = 0;
        int right_dist = 0;

        left_dist |= ((to_read[0] & 0x0F) << 10);
        left_dist |= ((to_read[1] & 0x7F) << 7);
        left_dist |= ((to_read[2] & 0x7F) << 0);
        if ((to_read[0] & 0x20) != 0)
        {
            left_dist *= -1;
        }

        right_dist |= ((to_read[3] & 0x0F) << 10);
        right_dist |= ((to_read[4] & 0x7F) << 7);
        right_dist |= ((to_read[5] & 0x7F) << 0);
        if ((to_read[3] & 0x20) != 0)
        {
            right_dist *= -1;
        }

        cout << "Left wheel went  " << left_dist << endl;
        cout << "Right wheel went " << right_dist << endl;

    

        //Publish the motion update
        //TODO: get this processed to a distance and an angle - code for this is in the python tests
        //msg.linear.x = right_dist;
        //msg.linear.y = left_dist;

        generate_motion_update(msg, right_dist, left_dist);

        pub.publish(msg);

//            usleep(100000);
        rate.sleep();


    }
    return 0;
    exit:
        close(file_i2c);


}
