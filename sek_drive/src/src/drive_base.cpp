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
#include <cmath> 

using namespace std;

#define PI 3.14159265358979323846264338
#define DIAMETER 0.1524 //m,6 INCHES
#define WHEEL_BASE_WIDTH 0.40 //m,  
#define TWOPI /*360*/6.2831853070
#define RADS 57.2958
#define MAX_SPEED 800 //command
#define MAX_RPM 360	//RPM
#define MAX_LIN_VEL  4
#define MIN_LIN_VEL  0.2
#define MAX_ROT_VEL  10
#define MIN_ROT_VEL  0.6

/*                                            
double circumference = PI* DIAMETER; //0.47877872040708448954
double max_lin_vel = circumference*(MAX_SPEED/60); //2.87267232244250693724
double max_ang_vle = max_lin_vel/(WHEEL_BASE_WIDTH/2);//0.14363361612212534686
int six = 0;
int gyro_speed = 0;
*/
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

int speed ;	
int count_ = 0;
int lenc = 0;
int renc = 0;

double posx = 0.0;
double posx_prev = 0.0;
double posy = 0.0;
double posy_prev = 0.0;
double posth = 0.0;
double linx = 0.0;
double angz = 0.0;
double linx_prev = 0.0;
double angz_prev = 0.0;
int battery = 0;

ros::Time current_time, last_time, total_time;

ofstream file;

unsigned int u;
FILE *output;

void teleopCallback(const sensor_msgs::Joy::ConstPtr& msg)
{	
    //current_time=ros::Time::now();
    
	if((msg->axes[5]!=0)||(msg->axes[6]!=0)||(msg->axes[0]!=0)||(msg->axes[1]!=0)||(msg->axes[4]!=0)||(msg->axes[3]!=0))
	{
		//ROS_INFO("HEAR HEAR");
		if(msg->axes[6]==1)//MPROS-PISW
		{
			RM = -MAX_SPEED;
			LM = MAX_SPEED;
            //gyro_speed = LM ;
		}
		else if(msg->axes[6]==-1)
		{
			RM = MAX_SPEED;
			LM = -MAX_SPEED;
            //gyro_speed = LM ;
            system("mplayer -really-quiet /home/skel/beep2.mp3&");
            
		}
		else if(msg->axes[5]!=0)//DEKSIA-ARISTERA
		{
			RM = -MAX_SPEED*msg->axes[5];
			LM = -MAX_SPEED*msg->axes[5];
		}
		else if((msg->axes[0]!=0)||(msg->axes[1]!=0))
        {
            if(msg->axes[1]==0)//akinito oxhma
            {
                if(msg->axes[0]!=0)//epitopia peristrofi
                {
                    RM = -MAX_SPEED*msg->axes[0];
                    LM = -MAX_SPEED*msg->axes[0];
                }
            }
            else
            {
                RM = -MAX_SPEED*msg->axes[1];
                LM = MAX_SPEED*msg->axes[1];
                if(msg->axes[1]>0)//kinisi mprosta
                {   
                    if(msg->axes[0]>0)//strofi aristera
                    {
                        RM = RM + RM*msg->axes[0];
                        if (RM<(-MAX_SPEED))
                        {
                            RM=-MAX_SPEED;
                        }
                        LM = LM - LM*msg->axes[0];
                        if (LM<(0.2*(MAX_SPEED)))
                        {
                            LM=0.2*(MAX_SPEED);
                        }
                        
                    }
                    else if (msg->axes[0]<0)//strofi de3ia
                    {
                        LM = LM - LM*msg->axes[0];
                        if (LM>(MAX_SPEED))
                        {
                            LM=MAX_SPEED;
                        }
                        RM = RM + RM*msg->axes[0];
                        if (RM>(-0.2*(MAX_SPEED)))
                        {
                            RM=-0.2*(MAX_SPEED);
                        }
                        
                    }
                }
                else if (msg->axes[1]<0)//kinisi pisw
                {   
                    RM = -MAX_SPEED*msg->axes[1]; //>0
                    LM = MAX_SPEED*msg->axes[1]; //<0
                    if(msg->axes[0]>0)//strofi aristera
                    {
                        RM = RM + RM*msg->axes[0];
                        if (RM>(MAX_SPEED))
                        {
                            RM=MAX_SPEED;
                        }
                        LM = LM - LM*msg->axes[0];
                        if (LM>(-0.2*(MAX_SPEED)))
                        {
                            LM=-0.2*(MAX_SPEED);
                        }
                        
                    }
                    else if (msg->axes[0]<0)//strofi de3ia
                    {
                        LM = LM - LM*msg->axes[0];
                        if (LM<(-MAX_SPEED))
                        {
                            LM = -MAX_SPEED;
                        }
                        RM = RM + RM*msg->axes[0];
                        if (RM<(0.2*(MAX_SPEED)))
                        {
                            RM=0.2*(MAX_SPEED);
                        }
                        
                    }
                }
            }
        }
        device.SetCommand(_GO,1, RM);// RIGHT
        device.SetCommand(_GO,2, LM);//LEFT
        cout<<"LEFT MOTOR :"<< LM<<endl;
        cout<<"RIGHT MOTOR :"<<RM<<endl;
        //calcOdom();
        if(msg->buttons[0]==1)
		{	
            system("mplayer -really-quiet /home/skel/horn.mp3 &");
            //ros::Duration(2).sleep();
            //system("killall -9 mplayer");
		}
        if(msg->buttons[3]==1)
        {
            ROS_INFO("SAVING MAP as \"mymap\"");
            system("rosrun map_server map_saver -f mymap &");
        }
    }
	
	else 
	{
        system("killall -9 mplayer");
		device.SetCommand(_GO,1, 0);// RIGHT
		device.SetCommand(_GO,2, 0);//LEFT
        
		if(msg->buttons[0]==1)
		{	
            system("mplayer -really-quiet /home/skel/horn.mp3 &");
            //ros::Duration(2).sleep();
            //system("killall -9 mplayer");
		}
        
        if(msg->buttons[3]==1)
        {
            ROS_INFO("SAVING MAP as \"mymap\"");
            system("rosrun map_server map_saver -f mymap &");
        }
        if(msg->buttons[2]==1)
        {
             system("mplayer -really-quiet /home/skel/blaster.mp3 &");
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
   

    //cout<<"RENC : "<<renc<<endl;
    //cout<<"LENC : "<<lenc<<endl;
    cout<<"LEFT MOTOR :"<< LM<<endl;
    cout<<"RIGHT MOTOR :"<<RM<<endl;
	
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{	
    //linx=(msg->linear.x / ((MAX_LIN_VEL + (MAX_LIN_VEL) / (1 - (0)))) + 1);
    //angz=(msg->angular.z / ((MAX_ROT_VEL + (MAX_ROT_VEL) / (1 - (0)))) + 1);
    if ((msg->linear.x==0)&&(msg->angular.z==0))
    {
        RM=0;
        LM=0;
        device.SetCommand(_GO,1, RM);// RIGHT
        device.SetCommand(_GO,2, LM);//LEFT
        cout<<"LEFT MOTOR :"<< LM<<endl;
        cout<<"RIGHT MOTOR :"<<RM<<endl;
        return;
    }
    //linx=(msg->linear.x / ((MAX_LIN_VEL + (MAX_LIN_VEL) / (1 - (0)))) + 1);
    //angz=(msg->angular.z / ((MAX_ROT_VEL + (MAX_ROT_VEL) / (1 - (0)))) + 1);
    linx=2 * (msg->linear.x + MAX_LIN_VEL)/( MAX_LIN_VEL + MAX_LIN_VEL) - 1;
    angz=2 * (msg->angular.z + MAX_ROT_VEL)/( MAX_ROT_VEL + MAX_ROT_VEL) - 1;
    linx_prev=linx;
    angz_prev=angz;
        //scale cmd_vel to [-1,1]
        cout<<"cmd_vel -> linear.x : "<<msg->linear.x<<endl;
        cout<<"cmd_vel -> angular.z : "<<msg->angular.z<<endl;
        cout<<"LINX : "<<linx<<endl;
        cout<<"ANGZ : "<<angz<<endl;
        if(linx==0)//akinito oxhma
            {
                if(angz!=0)//epitopia peristrofi
                {
                    RM = -MAX_SPEED*angz;
                    LM = -MAX_SPEED*angz;
                }
            }
            else
            {
                RM = -MAX_SPEED*linx;
                LM = MAX_SPEED*linx;
                if(linx>0)//kinisi mprosta
                {   
                    ROS_INFO("1");
                    if(angz>0)//strofi aristera
                    {
                        ROS_INFO("2");
                        RM = RM + RM*angz;
                        if (RM<(-MAX_SPEED))
                        {   ROS_INFO("3");
                            RM=-MAX_SPEED;
                        }
                        LM = LM - LM*angz;
                        if (LM<(0.2*(MAX_SPEED)))
                        {   ROS_INFO("4");
                            LM=0.2*(MAX_SPEED);
                        }
                        
                    }
                    else if (angz<0)//strofi de3ia
                    {   ROS_INFO("5");
                        LM = LM - LM*angz;
                        if (LM>(MAX_SPEED))
                        {   
                            ROS_INFO("6");
                            LM=MAX_SPEED;
                        }
                        RM = RM + RM*angz;
                        if (RM>(-0.2*(MAX_SPEED)))
                        {   
                            ROS_INFO("7");
                            RM=-0.2*(MAX_SPEED);
                        }
                        
                    }
                }
                else if (linx<0)//kinisi pisw
                {   
                    RM = -MAX_SPEED*linx; //>0
                    LM = MAX_SPEED*linx; //<0
                    if(angz>0)//strofi aristera
                    {
                        RM = RM + RM*angz;
                        if (RM>(MAX_SPEED))
                        {
                            RM=MAX_SPEED;
                        }
                        LM = LM - LM*angz;
                        if (LM>(-0.2*(MAX_SPEED)))
                        {
                            LM=-0.2*(MAX_SPEED);
                        }
                        
                    }
                    else if (angz<0)//strofi de3ia
                    {
                        LM = LM - LM*angz;
                        if (LM<(-MAX_SPEED))
                        {
                            LM = -MAX_SPEED;
                        }
                        RM = RM + RM*angz;
                        if (RM<(0.2*(MAX_SPEED)))
                        {
                            RM=0.2*(MAX_SPEED);
                        }
                        
                    }
                }
             }
        device.SetCommand(_GO,1, RM);// RIGHT
        device.SetCommand(_GO,2, LM);//LEFT
        cout<<"LEFT MOTOR :"<< LM<<endl;
        cout<<"RIGHT MOTOR :"<<RM<<endl;
}

void calcOdom()
{   
     if((device.GetValue(_S, 1, lenc)!=RQ_SUCCESS)||(device.GetValue(_S, 2, renc)!=RQ_SUCCESS))
		{
			ROS_INFO("Encoder data decoding failed\n");
            return;
		}  
	if(device.GetValue(_S, 2, renc)!=RQ_SUCCESS)
		{
            ROS_INFO("Encoder data decoding failed\n");
            return;
		}
    ROS_INFO("ODOM LENC : %d",lenc);
    ROS_INFO("ODOM RENC : %d",lenc);
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
	ros::Subscriber sub2 = n.subscribe("cmd_vel", 1, cmdVelCallback);
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

	//current_time = ros::Time::now();
	last_time = ros::Time::now();
while (ros::ok())
    {	
		ros::spinOnce();
        
     }   
	device.Disconnect();
	return 0;
}

