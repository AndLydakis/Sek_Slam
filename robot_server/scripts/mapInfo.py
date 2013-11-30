#! /usr/bin/env python
import numpy
import rospy
import roslib
import socket
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from robot_server.msg import chatter
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from PIL import Image
import os

class mapInfo(object):
    
    def __init__(self,socket):
        #publisher for starting point
        self.pose_pub=rospy.Publisher('initial_pose',PoseWithCovarianceStamped,latch=True)
        #pose message
        self.pose_msg=PoseWithCovarianceStamped()
        #publisher for destination point
        self.pose_pubDest=rospy.Publisher('goal_pose',PoseStamped,latch=True)
        #pose message
        self.pose_msgDest=PoseStamped()
        #matrix with quaternion coordinates
        self.quaternion=[0.0,0.0,0.0,0.0]
        self.socket=socket
        self.covariance = [0] * 36
        self.covariance[0]=0.25
        self.covariance[7]=0.25
        self.covariance[35]=0.06853891945200942
    
    def sendMap(self):
        size=self.convertImage()
        image = open('/home/skel/.ros/mymap.png', 'rb')
        imageData=image.read()
        try:
            self.socket.recv(32)
            print("size = {} ".format(size))
            #self.socket.sendall(str(size))
            #self.socket.recv(32)
            print("sending image")
            sent=self.socket.sendall(imageData)
            if(sent==None):
                print("image sent")
            else:
                print("failed to send")
            self.socket.recv(32)
        except socket.error as msg:
            print(msg)
            print("exception")
        pass
        
    def convertImage(self):
        path='/home/skel/.ros/mymap.pgm'
        newfile='/home/skel/.ros/mymap.png'
        if ( not os.path.isfile(newfile)):
            print("Error: {} file not found".format(newfile))
            x=Image.open(path)
            imsz=x.size
            print("size = {}".format(imsz))
            newimg=Image.new('L',imsz)
            newimg.putdata(x.getdata())
            newimg.save(newfile)
            print("image converted to png format ")
        return os.path.getsize(newfile)
    
    def setDestination(self):
        # arxikopoieitai to node
        #rospy.init_node('map_info_dest')
        s=self.socket
        try:
            s.sendall('1')
            self.quaternion[0]=(float(s.recv(64)))#receive x
            s.sendall('1')
            self.quaternion[1]=(float(s.recv(64)))#receive y
            s.sendall('1')
            self.quaternion[2]=(float(s.recv(64)))#receive z
            s.sendall('1')
            self.quaternion[3]=(float(s.recv(64)))#receive w
        except ValueError:
            print("Not a float")
            
        self.pose_msgDest.header.frame_id = 'map'
        self.pose_msgDest.pose.position.x=self.quaternion[0]
        self.pose_msgDest.pose.position.y=self.quaternion[1]
        self.pose_msgDest.pose.position.z=0.0
        
        self.pose_msgDest.pose.orientation.x=0.0
        self.pose_msgDest.pose.orientation.y=0.0
        self.pose_msgDest.pose.orientation.z=self.quaternion[2]
        self.pose_msgDest.pose.orientation.w=self.quaternion[3]
        
        self.pose_pubDest.publish(self.pose_msgDest)
        return 0

    
    def setStart(self):
        # arxikopoieitai to node
        rospy.init_node('map_info')
        s=self.socket
        try:
            s.sendall('1')
            self.quaternion[0]=(float(s.recv(64)))#receive x
            s.sendall('1')
            self.quaternion[1]=(float(s.recv(64)))#receive y
            s.sendall('1')
            self.quaternion[2]=(float(s.recv(64)))#receive z
            s.sendall('1')
            self.quaternion[3]=(float(s.recv(64)))#receive w
        except ValueError:
            print("Not a float")
        
        #self.pose_msg.header.stamp = rospy.get_rostime()#receive timestamp
        self.pose_msg.header.frame_id = 'map'
        self.pose_msg.pose.pose.position.x=self.quaternion[0]
        self.pose_msg.pose.pose.position.y=self.quaternion[1]
        self.pose_msg.pose.pose.position.z=0.0
        
        self.pose_msg.pose.pose.orientation.x=0.0
        self.pose_msg.pose.pose.orientation.y=0.0
        self.pose_msg.pose.pose.orientation.z=self.quaternion[2]
        self.pose_msg.pose.pose.orientation.w=self.quaternion[3]
        
        self.pose_msg.pose.covariance=self.covariance
        self.pose_pub.publish(self.pose_msg)
        #rospy.signal_shutdown("etsi")
        return 0

        
