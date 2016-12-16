#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition
from geometry_msgs.msg import PoseStamped

#Lots of globals to keep track of through all the functions
global pos0
global pos1
global Sub0
global Sub1
#global counter0
#global counter1

#Avarage the measured position 100 times before disconnecting the subscription
def callback0(data):
    global pos0
    #global counter0
    #if counter0==0:
    pos0=[data.pose.position.x, data.pose.position.y, data.pose.position.z]
    #else:
    #	pos0=[(pos0[0]+data.x)/2, (pos0[1]+data.y)/2, (pos0[2]+data.z)/2]
    #counter0+=1
    print pos0
    #if counter0>100:
    Sub0.unregister()

def callback1(data):
    global pos1
    #global counter1
    #if counter1==0:
    pos1=[data.pose.position.x, data.pose.position.y, data.pose.position.z]
    #else:
    #	pos1=[(pos1[0]+data.x)/2, (pos1[1]+data.y)/2, (pos1[2]+data.z)/2]
    #counter1+=1
    print pos1
    #if counter1>100:
    Sub1.unregister()

def shutdownhook():
    print "shutting down :D"

#Call the service set_target_position for both crazyflies to each others position
def Caller():
    setPosURI0 = "/crazyflie0/set_target_position" 
    setPosURI1 = "/crazyflie1/set_target_position"

    print "wait for services"

    rospy.wait_for_service(setPosURI0)
    rospy.wait_for_service(setPosURI1)
    
    setPos0=rospy.ServiceProxy(setPosURI0,SetTargetPosition)
    setPos1=rospy.ServiceProxy(setPosURI1,SetTargetPosition)

    print "set target positions:"
    print pos0
    print pos1 

    setPos0(pos1[0],pos1[1],pos1[2])
    setPos1(pos0[0],pos0[1],pos0[2])
    
    rospy.signal_shutdown("target positions sent")	#Ugly, but works

#Subscribes to the crazyflies positions by temporarly setting up a node
if __name__=='__main__':
    rospy.init_node('position_swapper')
    #global counter0
    #global counter1
    #counter0=0
    #counter1=0
    rospy.on_shutdown(shutdownhook)
    positionURI0 = "/crazyflie0/goal" 
    positionURI1 = "/crazyflie1/goal"
    Sub0=rospy.Subscriber(positionURI0, PoseStamped, callback0)
    Sub1=rospy.Subscriber(positionURI1, PoseStamped, callback1)
    while not rospy.is_shutdown():
	#callback becomes None when the subscription is unregistered
        if((Sub0.callback is None) and (Sub1.callback is None)):
            Caller()
