#!/usr/bin/env python

import rospy
from el2425_bitcraze.srv import SetTargetPosition
from geometry_msgs.msg import Point

global distance
global counter
global leader_cfId
global follower_cfId
#Call the service set_target_position for both crazyflies to each others position
def callback(data):
    global counter
    if counter>10:
        setPos=rospy.ServiceProxy("/crazyflie%s/set_target_position"%(follower_cfId),SetTargetPosition)
        setPos(data.x+distance[0], data.y+distance[1], data.z+distance[2])
        counter=0
    counter+=1


#Subscribes to the crazyflies positions by temporarly setting up a node
if __name__=='__main__':
    global counter
    global leader_cfId
    global follower_cfId
    leader_cfId=0
    follower_cfId=1
    counter=0
    rospy.init_node('distance_keeper')
    rate = rospy.Rate(5)
    global distance
    distance=[0.0,0.5,0.0]
    rospy.Subscriber("/crazyflie%s/crazyflie_position"%(leader_cfId), Point, callback)
    rospy.spin()
