#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
//XARALAMPOS
//#include <std_msgs/Int32MultiArray.h>
//std_msgs::Int32MultiArray //motor_commands;
//

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <tf/transform_broadcaster.h>
//#include <tf_conversions/tf_eigen.h>
#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>
#include "ros/ros.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <cmath> 

using namespace std;

#define PI 3.14159265358979323846264338
#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.40 //m,  
#define TWOPI /*360*/6.2831853070
#define RADS 57.2958
#define MAX_SPEED 1000 //command
#define MAX_LIN_VEL  2
#define MIN_LIN_VEL  -2
#define MAX_ROT_VEL  4
#define MIN_ROT_VEL  -4
#define ACCELERATION 80
int encoder_ppr = 455;
int encoder_cpr;
int LM = 0;
int RM = 0;
int lenc = 0, renc = 0, lenc_prev = 0, renc_prev = 0, lenc_init = 0, renc_init = 0, enc_errors = 0;
int MXRPM ;
double xx = 0, yy = 0, tt = 0;
bool firstOdom = true;
RoboteqDevice device;

ros::Time prev_time, current_time, enc_loop_time, bat_loop_time, tf_loop_time, mot_loop_time;
ros::Publisher odom_pub;
ros::Publisher pose_pub;
ros::Publisher pose_pub2;
tf::TransformBroadcaster *odom_broadcaster;

double wrapToPi(double angle)
{
    angle += M_PI;
    bool is_neg = (angle < 0);
    angle = fmod(angle, (2.0 * M_PI));
    if (is_neg) 
    {
        angle += (2.0 * M_PI);
    }
    angle -= M_PI;
    return angle;
}

void queryBat() 
{
    int status = -1;
    int battery_lvl;
    if((status =  device.GetValue(_VOLTS, 2, battery_lvl)) != RQ_SUCCESS)
    {
        ROS_ERROR("Error reading battary voltage");
    }
    else 
    {
        //ROS_INFO("BATTERY LEVEL : %d", battery_lvl/10);
    }
}

void readEnc()
{
    int status;
    if (!ros::ok() || !device.IsConnected())
        return;
    ros::Time now = ros::Time::now();
    double delta = (now - prev_time).toSec();
    /*
    if((status = device.GetValue(_ABSPEED, 1, renc)) !=RQ_SUCCESS)
    {
        cout<<"failed -->"<<status<<endl;
        ros::Duration(5.0).sleep();
        enc_errors++;
    }
    else
    {
        //cout<<"Right Motor Encoder : "<<renc<<endl;
        ros::Duration(0.1).sleep();
    }
    if((status = device.GetValue(_ABSPEED, 2, lenc)) !=RQ_SUCCESS)
    {
        cout<<"failed -->"<<status<<endl;
        ros::Duration(5.0).sleep();
        enc_errors++;
    }
    else
    {
        //cout<<"Left Motor Encoder : "<<lenc<<endl;
        ros::Duration(0.1).sleep();
    }
    */
    if((status = device.GetValue(_C, 1, renc)) !=RQ_SUCCESS)
    {
        cout<<"failed -->"<<status<<endl;
        ros::Duration(5.0).sleep();
        enc_errors++;
    }
    else
    {
        //cout<<"Right Motor Encoder : "<<renc<<endl;
        ros::Duration(0.1).sleep();
    }
    if((status = device.GetValue(_C, 2, lenc)) !=RQ_SUCCESS)
    {
        cout<<"failed -->"<<status<<endl;
        ros::Duration(5.0).sleep();
        enc_errors++;
    }
    else
    {
        //cout<<"Left Motor Encoder : "<<lenc<<endl;
        ros::Duration(0.1).sleep();
    }
    renc=-renc;
    lenc = lenc - lenc_init;
    renc = renc - renc_init;
    
}
void teleopCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    //ROS_INFO("lelelelel");
    
    int status;
    LM = msg->axes[1]*1000;
    RM = -msg->axes[3]*1000;
    device.SetCommand(_GO,1, RM);// RIGHT
    ros::Duration(0.01).sleep(); //sleep for 10 ms
    device.SetCommand(_GO,2, LM);//LEFT
    ros::Duration(0.01).sleep();
    if((status = device.SetCommand(_GO,1, RM)) != RQ_SUCCESS)
    {
      cout<<"failed --> "<<status<<endl;
      ros::Duration(5.0).sleep();
    }
    else
    {
      cout<<"MOTOR 1 SPEED  "<<RM<<endl;
    }
    if((status = device.SetCommand(_GO,2, LM)) !=RQ_SUCCESS)
    {
        cout<<"failed -->"<<status<<endl;
        ros::Duration(5.0).sleep();
    }
    else
    {
        cout<<"MOTOR 2 SPEED : "<<LM<<endl;
    }
}

void calcOdom()
{
    
    //cout<<"Odometry Callback"<<endl;
    ros::Time now = ros::Time::now();
    double delta_time = (now.toSec() - prev_time.toSec());
    //ROS_INFO("Delta Time : %f",delta_time);
    prev_time = now;
    
    if (firstOdom)
    {
        renc_prev = renc;
        lenc_prev = lenc;
    }
    
    int diff_lenc = lenc - lenc_prev;
    int diff_renc = renc - renc_prev;
    //int diff_lenc = lenc;
    //int diff_renc = renc;
    //ROS_INFO("encoder_cpr : %d",encoder_cpr);
    
    renc_prev = renc;
    lenc_prev = lenc;
    
    double l_w = diff_lenc * 2.0 * PI / (double)encoder_cpr / delta_time;
    double r_w = diff_renc * 2.0 * PI / (double)encoder_cpr / delta_time;
    ROS_INFO("L_W : %f     R_W : %f",l_w ,r_w);
    
    double l_v = l_w * DIAMETER / 2.0;
    double r_v = r_w * DIAMETER / 2.0;
    //ROS_INFO("L_V : %f     R_V : %f",l_v ,r_v);
    double v = (l_v + r_v) / 2.0;
    //ROS_INFO("V AFTER : %f",v);
    double w = (r_v - l_v) / WHEEL_BASE_WIDTH;
    
    /*
    double l_v = (lenc/60)*delta_time*2*M_PI*(DIAMETER/2);
    double r_v = -(renc/60)*delta_time*2*M_PI*(DIAMETER/2); 
    ROS_INFO("L_V : %f     R_V : %f",l_v ,r_v);
    double v = (l_v + r_v) / 2.0;
    //ROS_INFO("V AFTER : %f",v);
    double w = (r_v - l_v) / WHEEL_BASE_WIDTH;
    */
    geometry_msgs::Quaternion quat;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    tf::Quaternion q;
    
    if (firstOdom)
    {
        xx = 0;
        yy = 0;
        tt = 0;
        firstOdom = false;
    }
    else
    {
        tt = tt + w * delta_time;
        //ROS_INFO("TT BEFORE WRAP : %f",tt);
        tt = wrapToPi(tt);
        
    }
    //ROS_INFO("V BEFORE CREATE QUATERNION : %f",v);
    //ROS_INFO("TT AFTER WRAP : %f",tt);
    quat = tf::createQuaternionMsgFromRollPitchYaw(wrapToPi(roll), wrapToPi(pitch), tt);
    
    //ROS_INFO(" sdfgasdgasdga %f",v*cos(tt));
    xx = xx + (v * cos(tt)) * delta_time ;
    yy = yy + (v * sin(tt)) * delta_time;
    
    
    
    nav_msgs::Odometry odom_msg;
    //odom_msg.header.stamp = now;
    //odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = now;
    odom_msg.pose.pose.position.x = xx;
    ROS_INFO("%f",xx);
    odom_msg.pose.pose.position.y = yy;
    ROS_INFO("%f",yy);
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = w;

    odom_msg.pose.covariance[0] = 0.26298481867953083;
    odom_msg.pose.covariance[1] = -0.007138441796595563;
    odom_msg.pose.covariance[6] = -0.007138441796595563;
    odom_msg.pose.covariance[7] = 0.25096033123823897;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = 0.06684735983012981;
    odom_msg.twist.covariance = odom_msg.pose.covariance;

    odom_pub.publish(odom_msg);
    
    //Add TF broadcaster
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = xx;
    odom_trans.transform.translation.y = yy;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quat;
    odom_broadcaster->sendTransform(odom_trans);
    
    geometry_msgs::PoseStamped::Ptr pose_st_msg;
    pose_st_msg = boost::make_shared<geometry_msgs::PoseStamped>();
    pose_st_msg->header.stamp = now;
    pose_st_msg->header.frame_id = "odom";
            
            
    pose_st_msg->pose.position.x = xx;
    pose_st_msg->pose.position.y = yy;
    pose_st_msg->pose.position.z = 0;
    pose_st_msg->pose.orientation = odom_trans.transform.rotation;
            
    //tf::poseTFToMsg(odom_trans, pose_st_msg->pose);
    
    pose_pub2.publish(pose_st_msg);
}
    

int main(int argc, char *argv[])
{   
    int status = device.Connect("/dev/ttyACM1");
    if(status != RQ_SUCCESS)
    {
        cout<<"Error connecting to device: "<<status<<"."<<endl;
        return 1;
    }

    device.SetConfig(_RWD, 1, -1);
    device.SetConfig(_RWD, 2, -1);
    
    ros::init(argc, argv, "move_script");

    ros::NodeHandle n;
    prev_time = ros::Time::now();
    
    ros::Duration(0.1).sleep();
    odom_broadcaster = new tf::TransformBroadcaster;
    ros::Subscriber sub = n.subscribe("joy", 1, teleopCallback);
    odom_pub = n.advertise < nav_msgs::Odometry > ("/odom", 10);
    pose_pub2 = n.advertise<geometry_msgs::PoseStamped>("/poseStamped",5);
    //ros::Subscriber sub2 = n.subscribe("cmd_vel", 1, cmdVelCallback);
    //ros::Subscriber sub3 =  n.subscribe("amcl_pose", 1, recManCallback);
    //ros::Subscriber sub3 = n.subscribe("//motor_commands", 1, alignCallback);
    //ros::Publisher pub1 = n.advertise<nav_msgs::Path>("sent_path", 50);
    
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    //tf::TransformBroadcaster odom_broadcaster;
    
    //ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      //ROS_INFO("succeeded.");
      
    ros::Duration(0.01).sleep(); //sleep for 10 ms

    int result;
    int MAX_RPM;
    enc_loop_time = ros::Time::now();
    
    //ROS_INFO("- GetConfig(_DINA, 1)...");
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    
    //ROS_INFO("Reading RPM");
    if((status = device.GetConfig(_MXRPM, 1, MXRPM)) !=RQ_SUCCESS)
        cout<<"failed -->"<<status<<endl;
    else
        cout<<"MOTOR 1 MAXRPM : "<<MXRPM<<endl;
        
    if((status = device.GetConfig(_MXRPM, 2, MXRPM)) !=RQ_SUCCESS)
        cout<<"failed -->"<<status<<endl;
    else
        cout<<"MOTOR 2 MAXRPM : "<<MXRPM<<endl;
    
    encoder_cpr = encoder_ppr*4;
    //SET ACCELERATION
    /* 
    //ROS_INFO("SETTING ACCELERATION");
    if((status = device.SetConfig(_AC, 1, ACCELERATION)) !=RQ_SUCCESS)
        cout<<"failed -->"<<result<<endl;
    else
        cout<<"MOTOR 1 ACCEL : "<<ACCELERATION<<endl;
    
    if((status = device.SetConfig(_AC, 2, ACCELERATION)) !=RQ_SUCCESS)
        cout<<"failed -->"<<result<<endl;
    else
        cout<<"MOTOR 2 ACCEL : "<<ACCELERATION<<endl;
    
    
    //ROS_INFO("Roboteq -- Successfull setup of the Roboteq SDC2130");
    */
    printf ("Sek Operational\n\n");
    ros::Duration(0.01).sleep(); //sleep for 10 ms
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        if ((current_time.toSec() - enc_loop_time.toSec())>=0.3)
        {
            readEnc();
            calcOdom();
            enc_loop_time = current_time;
        }
        if ((current_time.toSec() - bat_loop_time.toSec())>=1)
        {
            queryBat();
            bat_loop_time = current_time;
        }
            
    }
    device.Disconnect();
    return 0;
}
