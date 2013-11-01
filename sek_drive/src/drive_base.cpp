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
#include <tf/transform_broadcaster.h>
#include "robodev.cpp"
#include <sdc2130_skel/RoboteqDevice.h>
#include <sdc2130_skel/ErrorCodes.h>
#include <sdc2130_skel/Constants.h>
#include "ros/ros.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>


using namespace std;

#define PI 3.14159265358979323846264338
#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.40 //m,  
#define TWOPI /*360*/6.2831853070
#define RADS 57.2958
#define MAX_SPEED 400 //command
#define MAX_RPM 360	//RPM

double circumference = PI* DIAMETER; //0.47877872040708448954
double max_lin_vel = circumference*(MAX_SPEED/60); //2.87267232244250693724
double max_ang_vle = max_lin_vel/(WHEEL_BASE_WIDTH/2);//0.14363361612212534686

int gyro_speed = 0;
int LM = 0 ;
int RM = 0 ;
int HB = 0 ; //handbrake signal
int ESD = 0 ; //emergency shutdown signal
int MODE = 0 ;
int REC = 0  ;
int status;
string response = "";
RoboteqDevice device;
int i ;	
int ping_ret,p_status;
//clock_t t;
int posleft;
int posright;
int rotR = 0;
int rotL = 0;
int rotRtot = 0;
int rotLtot = 0;
int speed ;	
int count_ = 0;
double posx = 0.0;
double posx_prev = 0.0;
double posy = 0.0;
double posy_prev = 0.0;
double posth = 0.0;

double mr = 0.0;
double ml = 0.0;
double mth = 0.0;
double mth_prev = 0.0;
double turn_deg = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
int distR = 0;
int distL = 0;
int total_dist = 0;
int battery = 0;
double seconds = 0.0;;
double deg = 0 ;
int dist = 0;
ros::Time current_time, last_time, total_time;
//ros::Time write_time, cur_write_time, total_time, cur2;
ofstream file;

unsigned int u;
FILE *output;

void teleopCallback(const sensor_msgs::Joy::ConstPtr& msg)
{	
	//current_time = ros::Time::now();
	int lenc,renc;
	if((msg->axes[5]!=0)||(msg->axes[6]!=0)||(msg->axes[0]!=0)||(msg->axes[1]!=0)||(msg->axes[4]!=0)||(msg->axes[3]!=0))
	{
		//device.SetConfig(_RWD, 1, -1);
		//device.SetConfig(_RWD, 2, -1);
		//ROS_INFO("HEAR HEAR");
		if(msg->axes[6]==1)//MPROS-PISW
		{
			RM = -MAX_SPEED;
			LM = MAX_SPEED;
            gyro_speed = LM ;
		}
		else if(msg->axes[6]==-1)
		{
			RM = MAX_SPEED;
			LM = -MAX_SPEED;
            gyro_speed = LM ;
		}
		else if(msg->axes[5]!=0)//DEKSIA-ARISTERA
		{
			RM = -MAX_SPEED*msg->axes[5];
			LM = -MAX_SPEED*msg->axes[5];
		}
		else if(msg->axes[1]!=0)
        {
            LM = msg->axes[1]*MAX_SPEED;
            RM = -msg->axes[1]*MAX_SPEED;
            speed = LM;
            if ((msg->axes[0]!=0)||(msg->axes[4]!=0))
            {
                if(speed==0)
                {
                    speed = 200;
                }
                if (msg->axes[0]>0)
                {
                    LM = - gyro_speed;
                    RM = - gyro_speed;
                }
                 else
                {
                    LM = gyro_speed;
                    RM = gyro_speed;
                }
            }
        }
        else if ((msg->axes[0]!=0)||(msg->axes[4]!=0))
        {
            if(speed==0)
            {
                gyro_speed = 200;
            }
            if (msg->axes[0]>0)
            {
                LM = - gyro_speed;
                RM = - gyro_speed;
            }
            else
            {
                LM = gyro_speed;
                RM = gyro_speed;
            }
        }
        device.SetCommand(_GO,1, RM);// RIGHT
        device.SetCommand(_GO,2, LM);//LEFT
    }
	
	else 
	{
		device.SetCommand(_GO,1, 0);// RIGHT
		device.SetCommand(_GO,2, 0);//LEFT
		if(msg->buttons[0]==1)
		{	

		}
		if((msg->buttons[10]==1)&&(msg->buttons[11]==1))
		{	
;
		}
		else if (msg->buttons[10]==1)
		{

		}
		else if (msg->buttons[11]==1)
		{

		}
		else if((msg->buttons[8]==1)&&(msg->buttons[9]==1))//SELECT & START
		{
			//device.SetConfig(_RWD, 1, 0);
			//device.SetConfig(_RWD, 2, 0);
			printf("Emergency Shutdown\n");			
			if((status = device.SetCommand(_GO, 2,  0) != RQ_SUCCESS))
			{
				cout<<"failed --> \n"<<status<<endl;
			}	
			else 
			{
			}
			ros::Duration(0.01).sleep(); 
			if((status = device.SetCommand(_GO, 1,- 0)) != RQ_SUCCESS)
			{
				cout<<"failed --> \n"<<status<<endl;
				device.Disconnect();
				ros::shutdown();
			}	
			else
			{
			}
			ROS_INFO("SHUTTING DOWN");
			ros::shutdown();
		}
	}
	/*
	if((device.GetValue(_S, 1, lenc)!=RQ_SUCCESS)||(device.GetValue(_S, 2, renc)!=RQ_SUCCESS))
		{
			ROS_INFO("Encoder data decoding failed\n");
		}  
		//if(device.GetValue(_S, 2, renc)!=RQ_SUCCESS)
		//	{
		//		ROS_INFO("Encoder data decoding failed\n");
		//	}
		else
		{	
			ROS_INFO("left_%d Right_%d",lenc,-renc);
			//ros::Duration(0.5).sleep();
			seconds= (current_time - last_time).toSec();
			mr = seconds*(-renc/60.0)*(DIAMETER*PI);
			ml = seconds*(lenc/60.0)*(DIAMETER*PI);
			//mr = -renc*(DIAMETER*PI);
			//ml = lenc*(DIAMETER*PI);
			dist=(ml+mr)/2.0;
			total_dist +=abs(dist);
			mth_prev = mth;
			mth += (ml-mr)/WHEEL_BASE_WIDTH;
			vth = (mth - mth_prev)/seconds;
			deg -=(float)((int)(mth/TWOPI))*TWOPI;
			posx_prev=posx;
			posy_prev=posy;
			ROS_INFO("COS %f",cos(mth*RADS));
			ROS_INFO("SIN %f",sin(mth*RADS));
			posx += (dist*(cos(mth*RADS)));
			posy += (dist*(sin(mth*RADS)));
			vx = (posx - posx_prev) / seconds;
			vy = (posy - posy_prev) / seconds;
			ROS_INFO("MR : %f",mr);
			ROS_INFO("ML : %f",ml);
			ROS_INFO("POSX : %f",posx);
			ROS_INFO("POSY : %f",posy);
			ROS_INFO("mth : %f",mth);
			ROS_INFO("VX : %f",vx);
			ROS_INFO("VY : %f",vy);
			ROS_INFO("VTH : %f",vth);
			mr=0;
			ml=0;
			dist=0;
			//last_time = current_time;
		}
		if(device.GetValue(_V,battery)!=RQ_SUCCESS)
		{
			ROS_INFO("Failed to read battery voltage\n");
		}
		*/
	
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{	
	/*
	# This expresses velocity in free space broken into its linear and angular parts.
	Vector3  linear
	Vector3  angular
	device.SetCommand(_GO,1, RM);// RIGHT anti8eta
	
	double linx =msg->linear.x;
	double liny =msg->linear.y;
	double linz =msg->linear.z;
	double angx =msg->angular.x;
	double angy =msg->angular.y;
	double angz =msg->angular.z;
	* */
	if (msg->linear.x!=0)
	{
		RM = (-msg->linear.x /*- msg->angular.z*WHEEL_BASE_WIDTH/2*/)*MAX_SPEED/2;
		LM = (msg->linear.x /*+ msg->angular.z*WHEEL_BASE_WIDTH/2*/)*MAX_SPEED/2;
		ROS_INFO("RM : %d" , RM);
		ROS_INFO("LM : %d" , LM);
		device.SetCommand(_GO,1, RM);
		device.SetCommand(_GO,2, LM);
	}
	else if (msg->angular.z!=0)
	{
		RM = (- msg->angular.z*WHEEL_BASE_WIDTH/2)*MAX_SPEED/2;
		LM = (- msg->angular.z*WHEEL_BASE_WIDTH/2)*MAX_SPEED/2;
		ROS_INFO("RM : %d" , RM);
		ROS_INFO("LM : %d" , LM);
		device.SetCommand(_GO,1, RM);
		device.SetCommand(_GO,2, LM);
	}
	else
	{
		device.SetCommand(_GO,1, 0);// RIGHT
		device.SetCommand(_GO,2, 0);//LEFT
	}
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
	ros::Duration(0.1).sleep();
	ros::Subscriber sub = n.subscribe("joy", 1, teleopCallback);
	//ros::Subscriber sub2 = n.subscribe("cmd_vel", 1, cmdVelCallback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	
	ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      ROS_INFO("succeeded.");
    ros::Duration(0.01).sleep(); //sleep for 10 ms

    int result;
    
    ROS_INFO("- GetConfig(_DINA, 1)...");
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    ROS_INFO("Roboteq -- Successfull setup of the Roboteq SDC2130");
    printf ("Sek Operational\n\n");
    ros::Duration(0.01).sleep(); //sleep for 10 ms

	current_time = ros::Time::now();
	last_time = ros::Time::now();
while (ros::ok())
    {	
		//current_time = ros::Time::now();
		ros::spinOnce();
        /*
        if((current_time.toSec() - last_time.toSec())>0.5)
        {
            ROS_INFO("HERE");
            p_status = system("ping -c 1 192.168.2.86");
            if (-1 != p_status)
            {
                ping_ret = WEXITSTATUS(p_status);
                if (ping_ret !=0)
                {
                    device.SetCommand(_GO,1, 0);// RIGHT
                    device.SetCommand(_GO,2, 0);//LEFT
                    device.Disconnect();
                    file.open ("/home/skel/ip_status.txt", ios::out | ios::binary);
                    file<<"DISCONNECTED"<<endl;
                    file.close();
                    ros::shutdown();
                    return 1;
                }
            }
            ROS_INFO("P_RET : %d" , ping_ret);
            last_time=ros::Time::now();
        }
        current_time=ros::Time::now();
		*/
     }   
	device.Disconnect();
	return 0;
}

