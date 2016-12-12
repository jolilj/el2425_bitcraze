#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty

# Enable the takeoff service. This assumes the crazyflie position is accurately estimated by the LPS system. 
# It might take a few seconds for the ekf to converge so wait before running this

if __name__ == "__main__":
    takeoffURI0 = "/crazyflie0/takeoff" 
    takeoffURI1 = "/crazyflie1/takeoff" 
    landURI0 = "/crazyflie0/land"
    landURI1 = "/crazyflie1/land"
    
    # Wait for services to become available
    rospy.loginfo("waiting for services")
    rospy.wait_for_service(takeoffURI0)
    rospy.wait_for_service(takeoffURI1)

    rospy.wait_for_service(landURI0)
    rospy.wait_for_service(landURI1)
    rospy.loginfo("found services")
    
    # Service call functions
    takeoff0 = rospy.ServiceProxy(takeoffURI0, Empty)
    land0 = rospy.ServiceProxy(landURI0, Empty)

    takeoff1 = rospy.ServiceProxy(takeoffURI0, Empty)
    land1 = rospy.ServiceProxy(landURI0, Empty)

    # Initial goal is already set from the position_handler, only need to takeoff.
    takeoff0()
    takeoff1()

    #Wait for user to press any key and then land
    raw_input("Press enter to land")
    land0()
    land1()

