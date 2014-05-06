#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

//#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>
#include <sek_drive/ARMarker.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 

using namespace std;

//marker coordinates relative to camera
double marker_pos_x_ = 0.0;
double marker_pos_y_ = 0.0;
double marker_pos_z_ = 0.0;
double marker_or_x_ = 0.0;
double marker_or_y_ = 0.0;
double marker_or_z_ = 0.0;
double marker_or_w_ = 0.0;
double PUBLISHED = 0;
double MARKER_FOUND = 0;
double RM, LM;
//turning direction 1 = left, -1 = right
int current_orientation = 1;
int ALIGNING = 0;
ros::Time time1_, time2_ ;

void markerCallback(const sek_drive::ARMarker::ConstPtr marker)
{
    marker_pos_x_ = marker->pose.pose.position.x;
    marker_pos_y_ = marker->pose.pose.position.y;
    marker_pos_z_ = marker->pose.pose.position.z;
    marker_or_x_ = marker->pose.pose.orientation.x;
    marker_or_y_ = marker->pose.pose.orientation.y;
    marker_or_z_ = marker->pose.pose.orientation.z;
    marker_or_w_ = marker->pose.pose.orientation.w;
    MARKER_FOUND = 1;
    time1_ = ros::Time::now();
    return;
}

void alignCallback(const std_msgs::Int32::ConstPtr align)
{
    ALIGNING = align->data;
}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "align_script");
	ros::NodeHandle n;
	ros::Duration(0.1).sleep();
	ros::Subscriber sub_ = n.subscribe("ar_pose_marker", 1, markerCallback);
    ros::Subscriber sub2_ = n.subscribe("align", 1, alignCallback);
    ros::Publisher pub1_ = n.advertise<std_msgs::Int32MultiArray>("motor_commands", 50);
    std_msgs::Int32MultiArray motor_commands;
    motor_commands.data.clear();
	tf::TransformBroadcaster marker_broadcaster_;
    cout<<"Starting Alignment"<<endl;
    ros::Duration(0.01).sleep();
    //ARMarker Default position is considered face up
    //when mounted on wall 
    //axis z : distance
    //axis x : angle, < 0 left side, > 0 right side
    while (ros::ok())
    {	
        ros::spinOnce();
		if (ALIGNING==1)
        {
            if (MARKER_FOUND == 1 )
            {   
                if ((ros::Time::now().toSec() - time1_.toSec()) > 0.1)
                {
                    MARKER_FOUND = 0;
                    ROS_INFO("MARKER LOST");
                }
                else
                {
                    if ((marker_pos_x_ < 0.005) && (marker_pos_x_ > -0.005))
                    {
                        ROS_INFO("CENTERED AR MARKER");
                        if(marker_pos_z_ > 0.30)
                        {
                            motor_commands.data.push_back(-160);
                            motor_commands.data.push_back(160);
                            ROS_INFO("MOVING FORWARD");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                            //PUBLISHED = 1;
                        }
                    }
                    else if (marker_pos_x_ < -0.005) //marker is at the right side of the camera
                    {
                        motor_commands.data.push_back(120);
                        motor_commands.data.push_back(-120);
                        ROS_INFO("ROTATING LEFT TO ALIGN");
                        pub1_.publish(motor_commands);
                        motor_commands.data.clear();
                        //PUBLISHED = 1;
                    }
                    else if (marker_pos_x_ > 0.005) //marker is at the left of the camera
                    {
                        motor_commands.data.push_back(-120);
                        motor_commands.data.push_back(120);
                        ROS_INFO("ROTATING RIGHT TO ALIGN");
                        pub1_.publish(motor_commands);
                        motor_commands.data.clear();
                        //PUBLISHED = 1;
                    }
                }
            }
        }
    }
	return 0;
}
