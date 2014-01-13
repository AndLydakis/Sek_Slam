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
#include <geometry_msgs/PoseStamped.h>
#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>
#include "ros/ros.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

int PATH_SENT = 0;
int FIRST_GOAL_SET = 0;
int GOAL_REACHED = 0;
int RESET = 0;
int GOAL_SENT = 0;

double X_Y_TOL = 0.1;
double YAW_TOL = 0.09;

int amcl_x = 0;
int amcl_y = 0;
int amcl_z = 0;
int amcl_w = 0;

int goal_x = 0;
int goal_y = 0;
int goal_z = 0;
int goal_w = 0;

std::vector<geometry_msgs::PoseStamped> waypoints, poses;
geometry_msgs::PoseStamped amcl_pose;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveClient;

//typedef SimpleClientGoalState MoveState;

bool isGoalReached()
{
    if(abs(goal_x-amcl_x) < X_Y_TOL)
    {
        if(abs(goal_y-amcl_y) < X_Y_TOL)
        {
            return true;
        }
    }
    return false;
}

void pathSentCallback(const nav_msgs::Path::ConstPtr& msg)
{
    waypoints.clear();
    poses = msg->poses;
    PATH_SENT = 1;
    FIRST_GOAL_SET = 0;
    system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &");
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = poses.begin() ; it!=poses.end(); ++it)
    {
        waypoints.push_back(*it);
    }
}

void cancelPathCallback(const std_msgs::Int64::ConstPtr& msg)
{
    if (msg->data==1)
    {
        PATH_SENT = 0;
        waypoints.clear();
    }
}


void amclCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    amcl_x = msg->pose.position.x;
    amcl_y = msg->pose.position.y;
    amcl_z = msg->pose.orientation.z;
    amcl_w = msg->pose.orientation.w;
}

/*
void initActionClients()
{
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac = new actionlib::SimpleActionClient("move_base", true);
}
*/
int main(int argc, char *argv[])
{	
    
    // while(!ac.waitForServer(ros::Duration(5.0)))
    ros::init(argc, argv, "path_sender");
    ROS_INFO("1");
    //ros::NodeHandle n;
    ROS_INFO("2");
    MoveClient ac("move_base", false);
    /*
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }    ROS_INFO("Created simpleActionClient");
    */
    //MoveState state ;
	//ros::Duration(0.1).sleep();
    ROS_INFO("3");
	//ros::Subscriber sub = n.subscribe("sent_path", 1, pathSentCallback);
    //ros::Subscriber sub2 = n.subscribe("cancel_path", 1, cancelPathCallback);
    //ros::Subscriber sub3 = n.subscribe("amcl_pose", 1, amclCallback);
    ROS_INFO("4");
	//ros::Publisher path_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base/goal", 50);
    ROS_INFO("5");
	std::vector<geometry_msgs::PoseStamped>::iterator it ;
    ROS_INFO("6");
    ROS_INFO("Creating simpleActionClient");
    
    ROS_INFO("Created SimpleClientGoalState"); 
    //move_base_msgs::MoveBaseGoal goal;
    ROS_INFO("WaitForResult"); 
    //ac.waitForResult();
    while (true)
    {	
        cout<<"wow so spin"<<endl;
        //ros::Duration(1).sleep();
		//ros::spinOnce(); 
        if (PATH_SENT==1)
        {
            if(FIRST_GOAL_SET==0)
            {
                if(waypoints.empty())
                {
                    PATH_SENT=0;
                    break;
                }
                cout<<"oeo"<<endl;
                it = waypoints.begin();
                cout<<"oeo"<<endl;
                FIRST_GOAL_SET=1;
                //cout << *it;
                it->header.frame_id = "map";
                //it->header.stamp = ros::Time::now();
                //path_pub.publish(*it);
                
                cout<<"oeo"<<endl;
                //goal_x = it->pose.position.x;
                cout<<"oeo"<<endl;
                //goal_y = it->pose.position.y;
                //goal_z = it->pose.orientation.z;
                //goal_w = it->pose.orientation.w;
                //cout<<"Goal (X Y Z)"<<goal_x<<" "<<goal_y<<" "<<goal_z<<" "<<goal_w<<endl;
            }
            else
            {
                //if ((state = ac.getState())== actionlib::SimpleClientGoalState::SUCCEEDED)
                if (isGoalReached)
                {
                    if(it==waypoints.end())
                    {
                        cout<<"Goal Reached !"<<endl;
                        PATH_SENT=0;
                        break;
                    }
                    it++;
                    cout << *it;
                    it->header.frame_id = "map";
                    //it->header.stamp = ros::Time::now();
                    //path_pub.publish(*it);
                    cout<<"oeo"<<endl;
                    goal_x = it->pose.position.x;
                    cout<<"oeo"<<endl;
                    goal_y = it->pose.position.y;
                    goal_z = it->pose.orientation.z;
                    goal_w = it->pose.orientation.w;
                    cout<<"Goal (X Y Z)"<<goal_x<<" "<<goal_y<<" "<<goal_z<<" "<<goal_w<<endl;
                    
                }
            }
        }
    }
	return 0;
}
