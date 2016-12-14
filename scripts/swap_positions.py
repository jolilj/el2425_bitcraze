#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray as Array
from std_srvs.srv import Empty

global pos0
global pos1
global Sub0
global Sub1
global counter0
global counter1

def callback0(data):
    pos0=[data.x, data.y, data.z]
    Sub0.unregister()
    print pos0

def callback1(data):
    pos1=[data.x, data.y, data.z]
    Sub1.unregister()
    print pos1

def shutdownhook():
    print "shutting down :D"

def Caller():
    setPosURI0 = "/crazyflie0/set_target_position" 
    setPosURI1 = "/crazyflie1/set_target_position"

    print "wait for services"

    rospy.wait_for_service(setPosURI0)
    rospy.wait_for_service(setPosURI1)
    
    setPos0=rospy.ServiceProxy(setPosURI0,SetTargetPosition)
    setPos1=rospy.ServiceProxy(setPosURI1,SetTargetPosition)

    print "set target positions"

    setPos0(pos1)
    setPos1(pos0)

if __name__=='__main__':
    rospy.init_node('position_swapper')
    rospy.on_shutdown(shutdownhook)
    positionURI0 = "/crazyflie0/crazyflie_position" 
    positionURI1 = "/crazyflie1/crazyflie_position"
    Sub0=rospy.Subscriber(positionURI0, Point, callback0)
    Sub1=rospy.Subscriber(positionURI1, Point, callback1)
    while not rospy.is_shutdown():
        #print Sub0.callback is None
        #print Sub1.callback is None
        if((Sub0.callback is None) and (Sub1.callback is None)):
            Caller()

