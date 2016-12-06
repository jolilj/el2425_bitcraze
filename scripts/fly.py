#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty

# Enable the takeoff service. This assumes the crazyflie position is accurately estimated by the LPS system. 
# It might take a few seconds for the ekf to converge so wait before running this

if __name__ == "__main__":
    # Wait for services to become available
    rospy.loginfo("waiting for services")
    rospy.wait_for_service('/crazyflie/takeoff')
    rospy.wait_for_service('/crazyflie/land')
    rospy.loginfo("found services")
    
    # Service call functions
    takeoff = rospy.ServiceProxy('/crazyflie/takeoff', Empty)
    land = rospy.ServiceProxy('/crazyflie/land', Empty)

    # Initial goal is already set from the position_handler, only need to takeoff.
    takeoff()

    #Wait for user to press any key and then land
    raw_input("Press any key to land")
    land()

