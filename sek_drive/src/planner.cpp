#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double x_,prevx_,y_,prevy_,z_,prevz_ ;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::Subscriber goal_sub = n.subscribe("cmd_vel", 1, cmdVelCallback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

 while(ros::ok())
 {
    
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal Reached");
  else
    ROS_INFO("Goal Unreachable");
  ros::spin();
  }
  return 0;
}
