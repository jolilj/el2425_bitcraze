#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

# Enable the takeoff service. This assumes the crazyflie position is accurately estimated by the LPS system. 
# It might take a few seconds for the ekf to converge so wait before running this

if __name__ == "__main__":
    rospy.loginfo("waiting for takeoff and landing service")
    rospy.wait_for_service('/crazyflie/takeoff')
    rospy.wait_for_service('/crazyflie/land')
    rospy.loginfo("found takeoff and landing service")

    takeoff = rospy.ServiceProxy('/crazyflie/takeoff', Empty)
    land = rospy.ServiceProxy('/crazyflie/land', Empty)

    takeoff()
    raw_input("Press any key to land")
    land()
