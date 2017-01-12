#!/usr/bin/env python

import rospy
from el2425_bitcraze.srv import SetTargetPosition
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

# ===== Simulate CF position ===
# Simple simulation: Assumes CF is at the goal position
# Publishes the goal points on the crazyflie_position topic #

posPub = rospy.Publisher(rospy.get_namespace() + "crazyflie_position", Point, queue_size=10, latch=True)

def callback(data):
    point = Point()
    point.x = data.pose.position.x
    point.y = data.pose.position.y
    point.z = data.pose.position.z
    posPub.publish(point)


if __name__=='__main__':
    rospy.init_node('position_sim')
    goalURI = rospy.get_namespace() + "goal" 
    rospy.Subscriber(goalURI, PoseStamped, callback)
    rospy.spin()

