#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>


void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
			file2<<odom->twist.twist.linear.x<<" "<<odom->twist.twist.linear.y<<" "<<odom->twist.twist.linear.z<<" "<<odom->twist.twist.angular.x<<" "<<odom->twist.twist.angular.y<<" "<<odom->twist.twist.angular.z<<" "<<endl;

}

int main(int argc, char** argv)
{	
    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;
    ros::Subscriber odom_in;
    ros::Publisher odom_pub;
    odom_in = n.subscribe("/"scanmatch_odom",1,odom_cb);
    while (ros::ok())
    {
        ros::spin();
	}
	return 0;
}
