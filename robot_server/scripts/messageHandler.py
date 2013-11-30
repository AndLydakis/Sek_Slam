from sensorControl import py_to_joy
from mapInfo import mapInfo
import threading
from threading import Thread
import os

class handler(object):

    #__socket=None
    run=False
    def __init__(self,socket):
        self.socket=socket
   
    def publish_key(self,key):
        while self.run:
            print(key)
        pass
    def sendMap(self):
        image = open('/home/skel/iit.png', 'rb')
        imageData=image.read()
        try:
            self.socket.recv(32)
            print("sending image")
            sent=self.socket.sendall(imageData)
            if(sent==None):
                print("image sent")
            else:
                print("failed to send")
        except socket.error as msg:
            print(msg)

    def handle(self):
        s=self.socket
        #diabase thn katastash sthn opoia 8elei na metabei o xrhsths
        #success=0
        while(True):
            try:
                state=int(s.recv(32))
                print(state)
            except ValueError:
                    print("Not a valid number at state")
            if (state==1):#metabash sthn katastash xeirismou xrhsimopoiontas koumpia
                print("Simple control")
                x=py_to_joy(s)
                x.controller()
                del x
            elif(state==2):#metabash sthn katastash xeirismou xrhsimopoiontas tous ais8hthres
                print("Sensor control")
                try:
                    turnSens=float(s.recv(64))
                    s.sendall("1")
                    moveSens=float(s.recv(64))
                except ValueError:
                    print("Not a valid float {} , {}".format(turnSens,moveSens))
                x=py_to_joy(s,turnSens,moveSens)
                x.sensor_controller()
                del x
            elif(state==3):
                x=mapInfo(s)
                x.sendMap()#Method to send the map
                del x
            elif(state==4):
                x=mapInfo(s)
                x.setStart()#set current location of the robot
                del x
            elif(state==5):
                x=mapInfo(s)
                x.setDestination()#set point to go
                del x
            elif(state==0):
                return False
            #elif
                #######
                #######
            #elif (state=0):
