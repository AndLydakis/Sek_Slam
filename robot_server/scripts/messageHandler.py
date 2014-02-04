from sensorControl import py_to_joy
from mapInfo import mapInfo
import threading
from threading import Thread
import os
import rospy
from RoboskelPos import RoboskelPos

class handler(object):

    run=False
    debug=True
    def __init__(self,socket):
        self.socket=socket
   
    def publish_key(self,key):
        while self.run:
            print(key)
        pass

    def handle(self):
        s=self.socket
        #diabase thn katastash sthn opoia 8elei na metabei o xrhsths
        #success=0
        rospy.init_node('python_to_joy')
        ##################
        x=mapInfo(s)
        x.garbage()#first publish is never published
        del x
        ##################
        z=None
        while(True):
            try:
                state=int(s.recv(32))
                if(self.debug):
                    print("State = {}".format(state))
            except ValueError:
                    print("Not a valid number at state {}|".format(state))
                    return False
            if (state==1):#metabash sthn katastash xeirismou xrhsimopoiontas koumpia
                if(self.debug):
                    print("simple control:State 1")
                x=py_to_joy(s)
                x.controller()
                del x
            elif(state==2):#metabash sthn katastash xeirismou xrhsimopoiontas tous ais8hthres
                if(self.debug):
                    print("sensor control:State 2")
                try:
                    turnSens=float(s.recv(64))
                    s.sendall('1')
                    moveSens=float(s.recv(64))
                except ValueError:
                    print("Not a valid float {} , {}".format(turnSens,moveSens))
                x=py_to_joy(s,turnSens,moveSens)
                x.sensor_controller()
                del x
            elif(state==3):#send the map
                if(self.debug):
                    print("send map:State 3")
                x=mapInfo(s)
                x.sendMap()
                del x
            elif(state==4):
                if(self.debug):
                    print("set location:State 4")
                x=mapInfo(s)
                x.setStart()#set current location of the robot
                del x
            elif(state==5):
                if(self.debug):
                    print("set destination:State 5")
                x=mapInfo(s)
                x.setDestination()#set point to go
                del x
            elif(state==6):#cancel automatic navigation
                if(self.debug):
                    print("cancel route:State 6")
                x=py_to_joy(s)
                x.cancelRoute()
                del x
            elif(state==7):#monitor robots movements
                if(self.debug):
                    print("movement feedback:state 7")
                if(z==None):
                    z=RoboskelPos(s)
                z.sendPoints()
            elif(state==8):#initiate camera steam
                s.sendall("1")
                if(self.debug):
                    print("stream:state 8")
                x=py_to_joy(s)
                x.stream()
            elif(state==9):
                if(self.debug):#execute saved pattern
                    print("begin maneuver:State 9")
                x=py_to_joy(s)
                x.beginManeuver()
                s.sendall("1")
            elif(state==10):#stop playback
                if(self.debug):
                    print("stop maneuver:State 10")
                x=py_to_joy(s)
                x.stopManeuver()
                s.sendall("1")
            elif(state==11):#record a pattern of moves
                if(self.debug):
                    print("begin recording:State 11")
                x=py_to_joy(s)
                x.beginRecording()
            elif(state==12):#stop recording pattern
                if(self.debug):
                    print("stop recording:State 12")
                x=py_to_joy(s)
                x.stopRecording()
            elif(state==13):#send multiple destination points
                if(self.debug):
                    print("multiple destination points")
                x=mapInfo(s)
                x.setMultDestinations()
            elif(state==0):
                print("Ending control")
                return False
            #elif
                #######
                #######
            #elif (state=0):
