//#include <iostream>
//#include <fstream>
//#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <ros/ros.h>
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
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 
//client includes
#include <netinet/in.h>
#include <resolv.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
////

//waypoints includes
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//

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
int ORDER_GIVEN = 0;
int MARKER_FOUND = 0;
int SEARCHING = 0;
int ALIGNING = 0;
int CUP_FILLED = 0;
int XY_THRES;
double YAW_THRES;
double RM, LM;

tf::Quaternion target_quat_, robot_quat_, robot_quat_rotated_;
ros::Time time1_, time2_ ;

geometry_msgs::PoseWithCovarianceStamped client_spot;
geometry_msgs::PoseWithCovarianceStamped coffee_spot;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
geometry_msgs::PoseWithCovariance marker_pose;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

/*HOW THIS SHOULD WORK
 * The robot receives the coffepot cordinates in a specific topic and navigates to it
 * When it is relatively in front of it it stops
 * It then starts rotating until the camera locate the AR marker
 * When the marker is in the frame the robot must slowly rotate 
 * and proceed towards it.
 * It is expected that the closer we get to the marker the slower the robot
 * will have to move in order to keep the marker centered (thresholds will need testing)
 * so maybe the speed will have to be adjusted inverse relatively to the distance from the marker
 * When the marker is at the center of the frame and at a certain distance (say 30 cm) the robot must send the signal
 * to the valve to open and then return to the client
*/
void returnToClient()
{
   return; 
}
void error(char* msg)
{
    perror(msg);
    exit(1);
}
bool inPosition()
{
    return ((marker_pos_x_ < 0.005) && (marker_pos_x_ > -0.005) && (marker_pos_z_ < 0.30));
}
void markerCallback(const sek_drive::ARMarker::ConstPtr marker_)
{
    //NOTIFIES WHEN THE MARKER IS IN THE CAMERA FRAME
    /*
    marker_pos_x_ = marker_->pose.pose.position.x;
    marker_pos_y_ = marker_->pose.pose.position.y;
    marker_pos_z_ = marker_->pose.pose.position.z;
    marker_or_x_ = marker_->pose.pose.orientation.x;
    marker_or_y_ = marker_->pose.pose.orientation.y;
    marker_or_z_ = marker_->pose.pose.orientation.z;
    marker_or_w_ = marker_->pose.pose.orientation.w;
    */
    marker_pose = marker_->pose;
    //tf::quaternionMsgToTF(marker_->pose.pose.orientation, target_quat_);
    if (CUP_FILLED == 0 )
    {
        if (MARKER_FOUND == 0)
        {
            ROS_INFO("MARKER FOUND");
        }
        MARKER_FOUND = 1;
    }
    time1_ = ros::Time::now();
    return;
}

void testPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& test_pose_)
{
    //USED FOR TESTING in order to specify a fake position for the robot
    //  in rl applications the robot position is taken from the amcl_pose topic
    //  from the navigation package
    /*
    robot_pos_x_ = test_pose_->pose.pose.position.x;
    robot_pos_y_ = test_pose_->pose.pose.position.y;
    robot_or_w_ = test_pose_->pose.pose.orientation.w;
    robot_or_z_ = test_pose_->pose.pose.orientation.z;
    */
    robot_pose.pose = test_pose_->pose;
    tf::quaternionMsgToTF(test_pose_->pose.pose.orientation, robot_quat_);
    ROS_INFO("ROBOT POSE RECEIVED");
    ROS_INFO("ROBOT X = %f", robot_pose.pose.pose.position.x);
    ROS_INFO("ROBOT Y = %f", robot_pose.pose.pose.position.y);
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION W = %f", robot_pose.pose.pose.orientation.w);
    ROS_INFO("ROBOT ORIENTATION Z = &f", robot_pose.pose.pose.orientation.z);
    if (CUP_FILLED = 0);
    {
        ALIGNING = 1;
    }
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose_)
{
    //FOR REAL TIME TESTING the robot calculates its position relatively to the 
    //specified coordinates for the coffee pot
    //if it is close enough it should cancel its current route and start the proccess of 
    //finding the pot
    /*
    robot_pos_x_ = amcl_pose_->pose.pose.position.x;
    robot_pos_y_ = amcl_pose_->pose.pose.position.y;
    robot_or_w_ = amcl_pose_->pose.pose.orientation.w;
    robot_or_z_ = amcl_pose_->pose.pose.orientation.z;
    */
    robot_pose.pose = amcl_pose_->pose;
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.header.frame_id = amcl_pose_->header.frame_id;
    tf::quaternionMsgToTF(amcl_pose_->pose.pose.orientation, robot_quat_);
    /*
    ROS_INFO("ROBOT POSE RECEIVED");
    ROS_INFO("ROBOT X = %f", robot_pos_x_);
    ROS_INFO("ROBOT Y = %f", robot_pos_y_);
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION X = 0.0");
    ROS_INFO("ROBOT ORIENTATION W = %f", robot_or_w_);
    ROS_INFO("ROBOT ORIENTATION Z = %f", robot_or_z_);
    */
    //if (sqrt((robot_pos_x_ - cof_x_)*(robot_pos_x_ - cof_x_) + (robot_pos_y_ - cof_y_)*(robot_pos_y_ - cof_y_)) < 0.30)
    if (sqrt((robot_pose.pose.pose.position.x - cof_x_)*(robot_pose.pose.pose.position.x - cof_x_) + (robot_pose.pose.pose.position.y - cof_y_)*(robot_pose.pose.pose.position.y - cof_y_)) < 0.30)
    {
        if (CUP_FILLED = 0);
        {
            system("rostopic pub -1 move_base/cancel actionlib_msgs/GoalID '{}' &"); //cancel its current action
            ALIGNING = 1;
        }
    }
}

void alignCallback(const std_msgs::Int32::ConstPtr& align)
{
    //FOR testing, let the robot know that it should start rotating
    ALIGNING = align->data;
}

void orderCallback(const std_msgs::Int32::ConstPtr& order)
{
    ORDER_GIVEN = order->data;
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "align_script");
    /*
    ROS_INFO("Creating MoveBaseClient");
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Connected to move base server");
    move_base_msgs::MoveBaseGoal goal;
    */
    ros::NodeHandle n;
    ros::Duration(0.1).sleep();
    ros::Subscriber sub_ = n.subscribe("ar_pose_marker", 1, markerCallback);
    ros::Subscriber sub2_ = n.subscribe("align", 1, alignCallback);
    ros::Subscriber sub3_ =  n.subscribe("amcl_pose", 1, poseCallback);
    ros::Subscriber sun4_ =  n.subscribe("test_pose", 1, testPoseCallback);
    ros::Subscriber sub5_ = n.subscribe("order_given", 1, orderCallback);
    ros::Publisher pub1_ = n.advertise<std_msgs::Int32MultiArray>("motor_commands", 50);
    //ros::Publisher pub2_ = n.advertise<geometry_msgs::PoseStampedWithCovariance>("move_base/simple_goal", 50);
    //SENDING OF GOALS CAN BE IMPLEMENTED WITH EITHER MESSAGES (SIMPLER)
    //OR ACTION_CLIENT(MORE CONTROL)
    
    //change the coffee pot location from the parameter server
    //or launch file
    n.getParam("align/xy_thres", XY_THRES);
    n.getParam("align/yaw_thres", YAW_THRES);
    n.getParam("align/cof_pos_x", cof_x_);
    n.getParam("align/cof_pos_y", cof_y_);
    n.getParam("align/cof_or_w", cof_or_w);
    n.getParam("align/cof_or_z", cof_or_z);
    n.getParam("align/aligning", ALIGNING);
    n.getParam("align/searching", SEARCHING);
    n.getParam("align/order_given", ORDER_GIVEN);
    //
    //ROS_INFO("TARGET X : %f", cof_x_);
    //ROS_INFO("TARGET Y : %f", cof_y_);
    //ROS_INFO("TARGET QW : %f", cof_or_w);
    //ROS_INFO("TARGET QZ : %f", cof_or_z);
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
    ROS_INFO("STARTING ALIGNMENT");
    cout<<"Starting Alignment"<<endl;
    ros::Duration(0.01).sleep();
    
    //client for connecting to valve
    int sock, lele;
    struct sockaddr_in server; 
    struct hostent *hp;
    char buff[256];
    //
    //ARMarker Default position is considered face up
    //when mounted on wall 
    //axis z : distance
    //axis x : angle, < 0 left side, > 0 right side
    while (ros::ok())
    {   /*DEBUGGING
        ROS_INFO("ROS_LOOP");
        ROS_INFO("ALIGNING : %d", ALIGNING);
        ROS_INFO("SEARCHING : %d", SEARCHING);
        ROS_INFO("MARKER_FOUND : %d", MARKER_FOUND);
        ROS_INFO("CUP_FILLED : %d", CUP_FILLED);
        ros::Duration(1.0).sleep();
        */
        ros::spinOnce();
        if (ORDER_GIVEN == 0)
        {
            if (n.getParam("/align/order_given", ORDER_GIVEN))
            {
                if (ORDER_GIVEN == 1)
                {
                    
                }
            }
        }
        //ROS_INFO("ORDER_GIVEN : %d", ORDER_GIVEN);
        //ros::Duration(3.0).sleep();
        if (ALIGNING==1)
        {   
            if (MARKER_FOUND == 1 )
            {   
                if(inPosition() && CUP_FILLED==0)
                {
                    ROS_INFO("FILLING CUP");
                    /*
                    CUP_FILLED = 1;
                    ALIGNING = 0;
                    SEARCHING = 0;
                    MARKER_FOUND = 0;
                    */
                    ros::Duration(3.0).sleep();
                    //CLIENT SERVER WAIT UNTIL CUP IS FULL
                    //server sends 0 when cup is full
                    sock = socket(AF_INET, SOCK_STREAM, 0);
                    if (sock <0) {
                        perror("SOCKET FAILED");
                        exit(1);
                    }
                    server.sin_family=AF_INET;
                    
                    if(inet_pton(AF_INET, "192.168.88.201", &server.sin_addr)<=0)
                    {
                        printf("\n inet_pton error occured\n");
                        return 1;
                    } 
                    
                    server.sin_port=htons(1234);
                    
                    if (connect(sock, (struct sockaddr *) &server, sizeof(server)) < 0) {
                        perror("connect failed");
                        close(sock);
                        exit(1);
                    }
                    
                    while (1) 
                    {
                        printf("Please enter the message: ");
                        bzero(buff,256);
                        fgets(buff,255,stdin);
                        lele = write(sock, buff, strlen(buff));
                        cout << "sent " << endl;
                        if (lele < 0) 
                        {
                         perror("ERROR writing to socket");
                         exit(1);
                        }
                        cout <<"reading" << endl;
                        lele = read(sock, buff, 256);
                        cout << "read :" << buff << endl;
                        if (lele < 0)
                        {
                            perror("ERROR reading from socket");
                            exit(1);
                        }
                        if(atoi(buff)==0)
                        {
                            cout << "shutting down" << endl;
                            close(sock);
                            //exit(1);
                            CUP_FILLED = 1;
                        }
                        ros::Duration(1.0).sleep();
                    } 
                    //printf("Sent %s\n", buff);
                    close(sock);
                    //
                    
                    motor_commands.data.push_back(0);
                    motor_commands.data.push_back(0);
                    pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                }
                if ((ros::Time::now().toSec() - time1_.toSec()) > 0.1) //check if the marker has left our f.o.v.
                {
                    MARKER_FOUND = 0;
                    previous_direction_ = current_direction_;
                    current_direction_ = 2;
                    motor_commands.data.push_back(0);
                    motor_commands.data.push_back(0);
                    pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                    ROS_INFO("MARKER LOST");
                }
                else
                {
                    if ((marker_pose.pose.position.x < 0.005) && (marker_pose.pose.position.x > -0.005))
                    //if ((marker_pos_x_ < 0.005) && (marker_pos_x_ > -0.005))
                    {
                        if (previous_direction_==3 && current_direction_==3)
                        {
                            ROS_INFO("CENTERED AR MARKER");
                            previous_direction_ = 0 ;
                            current_direction_ = 0;
                            motor_commands.data.push_back(-160);
                            motor_commands.data.push_back(160);
                            ROS_INFO("MOVING FORWARD");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                        else
                        {
                            current_direction_ = 0;
                            if ((marker_pose.pose.position.z > 0.30) && (current_direction_ != previous_direction_))
                            //if( (marker_pos_z_ > 0.30) && (current_direction_ != previous_direction_))
                            {
                                previous_direction_ = current_direction_;
                                ROS_INFO("CENTERED AR MARKER");
                                motor_commands.data.push_back(-160);
                                motor_commands.data.push_back(160);
                                ROS_INFO("MOVING FORWARD");
                                pub1_.publish(motor_commands);
                                motor_commands.data.clear();
                            }
                        }
                    }
                    else if (marker_pose.pose.position.x < - 0.005)
                    //else if (marker_pos_x_ < -0.005) //marker is at the right side of the camera
                    {
                        current_direction_ = -1;
                        if (current_direction_ != previous_direction_)
                        {
                            previous_direction_ = current_direction_ ;
                            motor_commands.data.push_back(-120);
                            motor_commands.data.push_back(-120);
                            ROS_INFO("ROTATING LEFT TO ALIGN");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                    }
                    else if (marker_pose.pose.position.x > 0.005)
                    //else if (marker_pos_x_ > 0.005) //marker is at the left of the camera
                    {
                        current_direction_ = 1;
                        if (current_direction_ != previous_direction_)
                        {
                            previous_direction_ = current_direction_ ;
                            //current_direction_ = 1;
                            motor_commands.data.push_back(120);
                            motor_commands.data.push_back(120);
                            ROS_INFO("ROTATING RIGHT TO ALIGN");
                            pub1_.publish(motor_commands);
                            motor_commands.data.clear();
                        }
                    }
                }
            }
            else if (SEARCHING == 0)
            {
                //AT THIS POINT the robot has reached the desired position 
                // and it may or may not be facing the marker
                // based on its estimated position and the given position of the pot it must
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
                    pub1_.publish(motor_commands);
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
                    pub1_.publish(motor_commands);
                    motor_commands.data.clear();
                }
                ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
                SEARCHING = 1;
            }
        }
    }
    return 0;
}
