
import rospy
import roslib
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from robot_server.msg import chatter
import threading
from threading import Thread


class py_to_joy(object):
    #the publisher
    key_pub=None
    #constructor
    def __init__(self,socket):
        #orizetai o subscriber poy akoyei sto topic chatter gia enan Integer
        self.chat_sub = rospy.Subscriber('chatter', Int32, self.chatter_cb,queue_size=10)
        #
        #self.chat_sub2 = rospy.Subscriber('chatter_2', chatter, self.chatter_cb2,queue_size=10)
        self.chat_sub2 = rospy.Subscriber('chatter_2', Float64MultiArray, self.chatter_cb2,queue_size=10)
        #orizetai o publisher poy 8a steilei ena minima Joy sto topic joy
        self.joy_pub = rospy.Publisher('joy', Joy)
        #orizetai to minima Joy poy 8a gemisoyme me dedomena
        self.joy_msg = Joy()
        #to minima Joy exei 2 pinakes gia dia8esima plhktra kai axones, to mege8os twn opoion orizoyme
        # me to extend edw estw 2 axones X,Y
        self.joy_msg.axes.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        self.joy_msg.buttons.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

        self.socket=socket
        
    def chatter_cb(self,data):
        x=data.data
        if(x==1):
            self.joy_msg.axes[6]=1
        elif(x==2):
            self.joy_msg.axes[5]=1
        elif(x==3):
            self.joy_msg.axes[6]=-1
        elif(x==4):
            self.joy_msg.axes[5]=-1
        elif(x==5):#user stops sending data
            self.joy_msg.axes[5]=0
            self.joy_msg.axes[6]=0
        elif(x==0):#user terminates
            self.joy_msg.axes[5]=0
            self.joy_msg.axes[6]=0
            reason="etsi"
            #rospy.signal_shutdown(reason)
        else:
            pass
        self.joy_pub.publish(self.joy_msg)
        self.joy_msg.axes[5]=0
        self.joy_msg.axes[6]=0
    #callback, kaleitai otan er8ei ena minima sto topic chatter
    
    def chatter_cb2(self,data):
        x=data.data
        #print("Pewpewpewpewpew")
        #return
        if(x[0]==12.0):
            pass
        elif(x[0]>=-1.0 and x[0]<=1.0):
            self.joy_msg.axes[0]=x[0]#right/left
        else:
            if(x[0]>0):
                self.joy_msg.axes[0]=1.0
            else:
                self.joy_msg.axes[0]=-1.0
        
        if(x[1]==12.0):
            pass
        elif(x[1]>=-1.0 and x[1]<=1.0):
            self.joy_msg.axes[1]=x[1]#up down
        else:
            if(x[1]>0):
                self.joy_msg.axes[1]=1.0
            else:
                self.joy_msg.axes[1]=-1.0
        
        self.joy_pub.publish(self.joy_msg)
        
        self.joy_msg.axes[0]=0.0
        self.joy_msg.axes[1]=0.0

    def controller(self):
        # arxikopoieitai to node
        rospy.init_node('python_to_joy')
        #orizetai o publisher poy 8a stelnei tis entoles
        self.key_pub = rospy.Publisher('chatter', Int32)
        while True:
            try:
                #diabazei thn entolh tou xrhsth apo to socket
                print("W8ting...")
                key=int(self.socket.recv(32))
                #i=threading.activeCount()
                #print i
            except ValueError:
                print("Not an integer")
            #an stal8ei h timh 0 termatizetai to module
            self.key_pub.publish(key)
            print("key ={}".format(key))
            if(key==0):
                break
        return 0
    
    def sensor_controller(self):
        # arxikopoieitai to node
        rospy.init_node('python_to_joy')
        #orizetai o publisher poy 8a stelnei tis entoles
        #self.key_pub = rospy.Publisher('chatter_2', chatter)
        self.key_pub2 = rospy.Publisher('chatter_2', Float64MultiArray)
        #self.socket.sendall(2);
        self.key_msg = Float64MultiArray()
        #oso leitoyrgei to node :
        key_d=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        while True:
            try:
                #diabazei thn entolh tou xrhsth apo to socket
                print("W8ting...")
                self.socket.sendall("Ready")
                #key[0]=float(self.socket.recv(64))
                key_d[0]=(float(self.socket.recv(64)))
                if(key_d[0]==12.0):
                    break
                self.socket.sendall("Ready")
                print("key[0] = {}".format(key_d[0]))
                #rospy.sleep(0.1)
                key_d[1]=float(self.socket.recv(64))
                #key.append(float(self.socket.recv(64)))
                #print("key[1] = {}".format(key[1]))
                #key=float(self.socket.recv(64))
                #i=threading.activeCount()
                #print i
                #for i in range (2,12):
                 #   key.append(0.0)
                 #   print key[i]
            except ValueError:
                print("Not a float")
            #an stal8ei h timh 0 termatizetai to module
            self.key_msg.data=key_d
            self.key_pub2.publish(self.key_msg)
            key_d=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            #print("key[0] = {} , key[1] = {}".format(key[0],key[1]))
            
        return 0        
