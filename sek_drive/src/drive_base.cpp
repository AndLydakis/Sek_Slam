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
#define MAX_ROT_VEL  4

double vlx = 2/(MAX_LIN_VEL - (-MAX_LIN_VEL)); //PARAGONTAS GIA SCALING TWN CMD_VEL ENTOLWN APO TO DIASTHMA
                                            //[-4,4] --> [-1,1]
double vrz = 2/(MAX_ROT_VEL - (-MAX_ROT_VEL));
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
/*
clock_t t;
int posleft;
int posright;
int rotR = 0;
int rotL = 0;
int rotRtot = 0;
int rotLtot = 0;
*/
int speed ;	
int count_ = 0;
int lenc = 0;
int renc = 0;

double posx = 0.0;
double posx_prev = 0.0;
double posy = 0.0;
double posy_prev = 0.0;
double posth = 0.0;
/*
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
*/
int battery = 0;
/*
double seconds = 0.0;
double deg = 0 ;
int dist = 0;
ros::Time write_time, cur_write_time, total_time, cur2;
*/
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
            if ((msg->axes[0]!=0)||(msg->axes[1]!=0))
            {   
                if (msg->axes[1] > 0)//eftheia
                {
                    LM = msg->axes[1]*MAX_SPEED;
                    RM = -msg->axes[1]*MAX_SPEED;
                    if (msg->axes[0] > 0)//aristera
                    {
                        if (msg->axes[1]==1)
                        {
                            LM = MAX_SPEED - MAX_SPEED*0.2;
                            RM = -MAX_SPEED ;
                        }
                        else
                        {
                            LM = LM - LM * fabs(msg->axes[0]);
                        }
                    }
                    else if (msg->axes[0]<0)//de3ia
                    {   
                        if (msg->axes[1]==1)
                        {
                            LM = MAX_SPEED;
                            RM = -MAX_SPEED + MAX_SPEED*0.2;
                        }
                        else
                        {
                            RM = RM - RM * fabs(msg->axes[0]);
                        }
                    }
                }
                else if (msg->axes[1] < 0)//pisw
                {
                    LM =  msg->axes[1]*MAX_SPEED;
                    RM = -msg->axes[1]*MAX_SPEED;
                    if (msg->axes[0] > 0)//aristera
                    {
                        if(msg->axes[1]==-1)
                        {
                            LM =  msg->axes[1]*MAX_SPEED;
                            RM = -msg->axes[1]*MAX_SPEED - MAX_SPEED*0.2; 
                        }
                        else
                        {
                            LM = LM - LM * fabs(msg->axes[0]);
                        }
                    }
                    else if (msg->axes[0]<0)//de3ia
                    {   
                        if(msg->axes[1]==-1)
                        {
                            LM =  msg->axes[1]*MAX_SPEED - MAX_SPEED*0.2;
                            RM = -msg->axes[1]*MAX_SPEED ; 
                        }
                        else
                        {
                            RM = RM - RM * fabs(msg->axes[0]);
                        }
                    }
                }
                else//akinhto oxhma
                {   
                    //cout<<"AKINHTO"<<endl;
                    if (msg->axes[0] > 0)//aristera
                    {   
                        LM = -MAX_SPEED*msg->axes[0];
                        RM = -MAX_SPEED*msg->axes[0];
                        /*
                        if(msg->axes[0]==0.5)
                        {
                            LM = -MAX_SPEED/2;
                            RM = -MAX_SPEED/2;
                        }
                        else if(msg->axes[0] < 0.5)
                        {   
                            //cout<<"IF 1"<<endl;
                            LM = (MAX_SPEED)*(msg->axes[0]);
                            RM = -(MAX_SPEED)*(1-msg->axes[0]);
                        }
                        else
                        {
                            //cout<<"ELSE 1"<<endl;
                            LM = (MAX_SPEED)*(1-fabs(msg->axes[0]));
                            RM = -(MAX_SPEED)*(fabs(msg->axes[0]));
                        }
                        */
                    }
                    else if (msg->axes[0] < 0)//de3ia
                    {
                        LM = -MAX_SPEED*msg->axes[0];
                        RM = -MAX_SPEED*msg->axes[0];
                        /*
                        if(msg->axes[0]==-0.5)
                        {
                            LM = MAX_SPEED/2;
                            RM = MAX_SPEED/2;
                        }
                        else if (msg->axes[0] > - 0.5)
                        {   
                            //cout<<"IF2"<<endl;
                            LM = (MAX_SPEED)*(1-fabs(msg->axes[0]));
                            RM = -(MAX_SPEED)*(fabs(msg->axes[0]));
                            
                        }
                        else
                        {
                            //cout<<"ELSE2"<<endl;
                            LM = (MAX_SPEED)*(fabs(msg->axes[0]));
                            RM = -(MAX_SPEED)*(1-fabs(msg->axes[0]));
                        }
                        */
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
    double linx =msg->linear.x;
    if (msg->linear.x!=0)
	{
		RM = (-msg->linear.x*2 - 1/*- msg->angular.z*WHEEL_BASE_WIDTH/2*/)*MAX_SPEED;
		LM = (msg->linear.x*2 - 1/*+ msg->angular.z*WHEEL_BASE_WIDTH/2*/)*MAX_SPEED;
		ROS_INFO("RM : %d" , RM);
		ROS_INFO("LM : %d" , LM);
		device.SetCommand(_GO,1, RM);
		device.SetCommand(_GO,2, LM);
        if(msg->angular.z!=0)
        {
            if(msg->angular.z > 0)
            {
                LM = LM - LM * fabs(msg->angular.z);
            }
            else if(msg->angular.z < 0)
            {
                RM = RM - RM * fabs(msg->angular.z);
            }
        }
	}
	else if (msg->angular.z!=0)
	{
		RM = (- msg->angular.z*2 - 1)*MAX_SPEED/2;
		LM = (- msg->angular.z*2 - 1)*MAX_SPEED/2;
		ROS_INFO("RM : %d" , RM);
		ROS_INFO("LM : %d" , LM);
		device.SetCommand(_GO,1, RM);
		device.SetCommand(_GO,2, LM);
	}
	else
	{
		device.SetCommand(_GO,1, 0);// RIGHT
		device.SetCommand(_GO,2, 0);//LEFT
        cout<<"ARISTERO MOTORI :"<< LM<<endl;
        cout<<"DEKSIO MOTORI :"<<RM<<endl;
	}
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
/*
void calcOdom_()
{
    ROS_INFO("left_%d Right_%d",lenc,-renc);
    //ros::Duration(0.5).sleep();
    seconds= (current_time - last_time).toSec();
    mr = seconds*(-renc/60.0)*(DIAMETER*PI);
    ml = seconds*(lenc/60.0)*(DIAMETER*PI);
    //mr = -renc*(DIAMETER*PI);
    //ml = lenc*(DIAMETER*PI);
    dist=(ml+mr)/2.0;
    total_dist +=fabs(dist);
    mth_prev = mth;
    mth += (ml-mr)/WHEEL_BASE_WIDTH;
    vth = (mth - mth_prev)/seconds;
    deg -=(float)((int)(mth/TWOPI))*TWOPI;
    posx_prev=posx;
    posy_prev=posy;
    //ROS_INFO("COS %f",cos(mth*RADS));
    //ROS_INFO("SIN %f",sin(mth*RADS));
    posx += (dist*(cos(mth*RADS)));
    posy += (dist*(sin(mth*RADS)));
    vx = (posx - posx_prev) / seconds;
    vy = (posy - posy_prev) / seconds;
    //ROS_INFO("MR : %f",mr);
    //ROS_INFO("ML : %f",ml);
    //ROS_INFO("POSX : %f",posx);
    //ROS_INFO("POSY : %f",posy);
    //ROS_INFO("mth : %f",mth);
    //ROS_INFO("VX : %f",vx);
    //ROS_INFO("VY : %f",vy);
    //ROS_INFO("VTH : %f",vth);
    mr=0;
    ml=0;
    dist=0;
}
*/
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
        /*
        current_time = ros::Time::now();
        seconds = current_time.toSec() - last_time.toSec();
        //ROS_INFO("SECONDS : %f",seconds);
        if(seconds > 0.2)
        {
            //ROS_INFO("IN LOOP");
            //ros::Duration(3).sleep();
            calcOdom();
            last_time = ros::Time::now();
            current_time = ros::Time::now();

        }
        */
     }   
	device.Disconnect();
	return 0;
}

