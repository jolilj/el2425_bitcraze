#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(data)

def listener():
	rospy.init_node('position_logger')
	rospy.Subscribe("/crazyflie/crazyflie_position",String,callback)
	rospy.spin()
