#!/usr/bin/env python

import rospy
from el2425_bitcraze.srv import SetTargetPosition
from geometry_msgs.msg import Point

global distance

#Call the service set_target_position for both crazyflies to each others position
def callback(data):
    setPos=rospy.ServiceProxy("/crazyflie1/set_target_position",SetTargetPosition)
    setPos(data.x+distance[0], data.y+distance[1], data.z+distance[2])


#Subscribes to the crazyflies positions by temporarly setting up a node
if __name__=='__main__':
    rospy.init_node('distance_keeper')
    rate = rospy.Rate(5)
    global distance
    distance=[0,0.5,0]
    rospy.Subscriber("/crazyflie0/crazyflie_position", Point, callback)
    rospy.spin()
    #while not rospy.is_shutdown():
    #    rate.sleep()
