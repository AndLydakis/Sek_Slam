#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>
#include "ros/ros.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 

geometry_msgs::PoseWithCovarianceStamped init_pose;
int sent=0;

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    init_pose.pose=msg->pose;
    init_pose.header.frame_id = "/map";
    sent=1;
}

int main(int argc, char *argv[])
{	
    ros::init(argc, argv, "move_script");
    ros::NodeHandle n;
	ros::Duration(0.1).sleep();
    //ros::Subscriber sub4 = n.subscribe("sent_in_pose", 1, initPoseCallback);
	ros::Publisher path_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5,true);
    ros::Subscriber sub = n.subscribe("initial_pose", 1, initPoseCallback);

   
   while (ros::ok())
    {	
        if (sent==0)
        {
            cout<<"oeo"<<endl;
            init_pose.header.frame_id = "/map";
            init_pose.pose.pose.position.x=-0.35;
            init_pose.pose.pose.position.y=0.415857434273;
            init_pose.pose.pose.position.z=0.0;
                
            init_pose.pose.pose.orientation.x=0.0;
            init_pose.pose.pose.orientation.y=0.0;
            init_pose.pose.pose.orientation.z=-0.0179285788903;
            init_pose.pose.pose.orientation.w= 0.999839270112;
            
            init_pose.pose.covariance[0]=0.25;
            init_pose.pose.covariance[7]=0.25;
            init_pose.pose.covariance[35]=0.06853891945200942;
            path_pub.publish(init_pose);
    
            sent=1;
       
        }
        ros::spinOnce();
        if(sent==1)
        {
            //ros::shutdown();
        }
   } 
    return 0;
}
