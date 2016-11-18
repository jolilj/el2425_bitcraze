#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

# Enable the takeoff service. This assumes the crazyflie position is accurately estimated by the LPS system. 
# It might take a few seconds for the ekf to converge so wait before running this

if __name__ == "__main__":
	rospy.loginfo("waiting for takeoff service")
	rospy.wait_for_service('/crazyflie/takeoff')
	rospy.loginfo("found takeoff service")
	takeoff = rospy.ServiceProxy('/crazyflie/takeoff', Empty)
	takeoff()
