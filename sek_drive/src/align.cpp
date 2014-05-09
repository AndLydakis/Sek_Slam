#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <resource_retriever/retriever.h>
#include <sek_drive/ARMarker.h>
#include <tf/transform_datatypes.h>
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
//robot coordinates
double robot_pos_x_ = 0.0;
double robot_pos_y_ = 0.0;
double robot_or_w_ = 0.0;
double robot_or_z_ = 0.0;
//target coordinates
double cof_x_, cof_y_, cof_or_w, cof_or_z;
//turning direction 1 = left, -1 = right, 0 = straight, 2 stopped
int current_direction_ = 3;
int previous_direction_ = 3;
int PUBLISHED = 0;
int MARKER_FOUND = 0;
int SEARCHING = 0;
int ALIGNING = 0;
int RM, LM;

tf::Quaternion target_quat_, robot_quat_, robot_quat_rotated_;
ros::Time time1_, time2_ ;

void markerCallback(const sek_drive::ARMarker::ConstPtr marker_)
{
    marker_pos_x_ = marker_->pose.pose.position.x;
    marker_pos_y_ = marker_->pose.pose.position.y;
    marker_pos_z_ = marker_->pose.pose.position.z;
    marker_or_x_ = marker_->pose.pose.orientation.x;
    marker_or_y_ = marker_->pose.pose.orientation.y;
    marker_or_z_ = marker_->pose.pose.orientation.z;
    marker_or_w_ = marker_->pose.pose.orientation.w;
    //tf::quaternionMsgToTF(marker_->pose.pose.orientation, target_quat_);
    if (MARKER_FOUND == 0)
    {
        ROS_INFO("MARKER FOUND");
    }
    MARKER_FOUND = 1;
    time1_ = ros::Time::now();
    return;
}

void testPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& test_pose_)
{
    robot_pos_x_ = test_pose_->pose.pose.position.x;
    robot_pos_y_ = test_pose_->pose.pose.position.y;
    robot_or_w_ = test_pose_->pose.pose.orientation.w;
    robot_or_z_ = test_pose_->pose.pose.orientation.z;
    tf::quaternionMsgToTF(test_pose_->pose.pose.orientation, robot_quat_);
    ROS_INFO("ROBOT POSE RECEIVED");
    ROS_INFO("ROBOT X = %f", robot_pos_x_);
    ROS_INFO("ROBOT Y = %f", robot_pos_y_);
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION W = %f", robot_or_w_);
    ROS_INFO("ROBOT ORIENTATION Z = &f", robot_or_z_);
    ALIGNING = 1;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_)
{
    robot_pos_x_ = amcl_pose_->pose.pose.position.x;
    robot_pos_y_ = amcl_pose_->pose.pose.position.y;
    robot_or_w_ = amcl_pose_->pose.pose.orientation.w;
    robot_or_z_ = amcl_pose_->pose.pose.orientation.z;
    tf::quaternionMsgToTF(amcl_pose_->pose.pose.orientation, robot_quat_);
    ROS_INFO("ROBOT POSE RECEIVED");
    ROS_INFO("ROBOT X = %f", robot_pos_x_);
    ROS_INFO("ROBOT Y = %f", robot_pos_y_);
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION W = %f", robot_or_w_);
    ROS_INFO("ROBOT ORIENTATION Z = %f", robot_or_z_);
    //if (sqrt((robot_pos_x_ - cof_x_)*(robot_pos_x_ - cof_x_) + (robot_pos_y_ - cof_y_)*(robot_pos_y_ - cof_y_)) < 0.30)
    //{
        ALIGNING = 1;
    //}
}

void alignCallback(const std_msgs::Int32::ConstPtr& align)
{
    ALIGNING = align->data;
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "align_script");
    ros::NodeHandle n;
    ros::Duration(0.1).sleep();
    //ros::Subscriber sub_ = n.subscribe("ar_pose_marker", 1, markerCallback);
    ros::Subscriber sub2_ = n.subscribe("align", 1, alignCallback);
    ros::Subscriber sub3_ =  n.subscribe("amcl_pose", 1, poseCallback);
    //ros::Subscriber su4_ =  n.subscribe("test_pose", 1, testPoseCallback);
    ros::Publisher pub1_ = n.advertise<std_msgs::Int32MultiArray>("motor_commands", 50);
    //target coordinates from parameter server
    n.getParam("align/cof_pos_x", cof_x_);
    n.getParam("align/cof_pos_y", cof_y_);
    n.getParam("align/cof_or_w", cof_or_w);
    n.getParam("align/cof_or_z", cof_or_z);
    //
    ROS_INFO("TARGET X : %f", cof_x_);
    ROS_INFO("TARGET Y : %f", cof_y_);
    ROS_INFO("TARGET QW : %f", cof_or_w);
    ROS_INFO("TARGET QZ : %f", cof_or_z);
    geometry_msgs::Quaternion final_quat;
    geometry_msgs::Quaternion temp_quat_;
    double roll, pitch, yaw;
    tf::Quaternion rotation_quat_;
    temp_quat_.x = 0.0;
    temp_quat_.y = 0.0;
    temp_quat_.w = cof_or_w;
    temp_quat_.z = cof_or_z;
    tf::quaternionMsgToTF(temp_quat_, target_quat_);
    temp_quat_.x = 0.0;
    temp_quat_.y = 0.0;
    temp_quat_.w = 1.0;
    temp_quat_.z = 0.0;
    tf::quaternionMsgToTF(temp_quat_, rotation_quat_);
    target_quat_ = target_quat_ * rotation_quat_;
    std_msgs::Int32MultiArray motor_commands;
    motor_commands.data.clear();
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
                if (SEARCHING == 1)
                {
                    motor_commands.data.push_back(0);
                    motor_commands.data.push_back(0);
                    //pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                }
                SEARCHING = 0;
                if ((ros::Time::now().toSec() - time1_.toSec()) > 0.1)
                {
                    MARKER_FOUND = 0;
                    previous_direction_ = current_direction_;
                    current_direction_ = 2;
                    motor_commands.data.push_back(0);
                    motor_commands.data.push_back(0);
                    //pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                    ROS_INFO("MARKER LOST");
                }
                else
                {
                    if ((marker_pos_x_ < 0.005) && (marker_pos_x_ > -0.005))
                    {
                        if (previous_direction_==3 && current_direction_==3)
                        {
                            ROS_INFO("CENTERED AR MARKER");
                            previous_direction_ = 0 ;
                            current_direction_ = 0;
                            motor_commands.data.push_back(-160);
                            motor_commands.data.push_back(160);
                            ROS_INFO("MOVING FORWARD");
                            //pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                        else
                        {
                            current_direction_ = 0;
                            if( (marker_pos_z_ > 0.30) && (current_direction_ != previous_direction_))
                            {
                                previous_direction_ = current_direction_;
                                ROS_INFO("CENTERED AR MARKER");
                                motor_commands.data.push_back(-160);
                                motor_commands.data.push_back(160);
                                ROS_INFO("MOVING FORWARD");
                                //pub1_.publish(motor_commands);
                                motor_commands.data.clear();
                                //PUBLISHED = 1;
                            }
                        }
                    }
                    else if (marker_pos_x_ < -0.005) //marker is at the right side of the camera
                    {
                        current_direction_ = -1;
                        if (current_direction_ != previous_direction_)
                        {
                            previous_direction_ = current_direction_ ;
                            motor_commands.data.push_back(-120);
                            motor_commands.data.push_back(-120);
                            ROS_INFO("ROTATING LEFT TO ALIGN");
                            //pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                            //PUBLISHED = 1;
                        }
                    }
                    else if (marker_pos_x_ > 0.005) //marker is at the left of the camera
                    {
                        current_direction_ = 1;
                        if (current_direction_ != previous_direction_)
                        {
                            previous_direction_ = current_direction_ ;
                            //current_direction_ = 1;
                            motor_commands.data.push_back(120);
                            motor_commands.data.push_back(120);
                            ROS_INFO("ROTATING RIGHT TO ALIGN");
                            //pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                            //PUBLISHED = 1;
                        }
                    }
                }
            }
            else if (SEARCHING == 0)
            {
                //caclulate the direction for the shortest turn 
                robot_quat_rotated_ = target_quat_ * robot_quat_.inverse();
                tf::Matrix3x3(robot_quat_rotated_).getRPY(roll, pitch, yaw);
                if (yaw < 0)
                {
                    //rotate left
                    robot_quat_rotated_.setZ(-robot_quat_rotated_.getZ());
                    ROS_INFO("YAW = %f", yaw);
                    //ros::Duration(5.0).sleep();
                    motor_commands.data.push_back(-120);
                    motor_commands.data.push_back(-120);
                    ROS_INFO("SEARCHING FOR MARKER");
                    //pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                }
                else
                {   
                    //rotate right
                    ROS_INFO("YAW = %f", yaw);
                    //ros::Duration(5.0).sleep();
                    motor_commands.data.push_back(120);
                    motor_commands.data.push_back(120);
                    ROS_INFO("SEARCHING FOR MARKER");
                    //pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                }//
                ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
                SEARCHING = 1;
                
            }
        }
    }
    return 0;
}
