#!/usr/bin/env python

import rospy
from el2425_bitcraze.srv import SetPolygonTrajectory 


if __name__ == "__main__":
    trajectory0URI = "/crazyflie0/set_polygon_trajectory"
    trajectory1URI = "/crazyflie1/set_polygon_trajectory"
    
    # Wait for services to become available
    rospy.loginfo("waiting for services")
    rospy.wait_for_service(trajectory0URI)
    rospy.wait_for_service(trajectory1URI)
    rospy.loginfo("found services")

    setTrajectory0 = rospy.ServiceProxy(trajectory0URI, SetPolygonTrajectory)
    setTrajectory1 = rospy.ServiceProxy(trajectory1URI, SetPolygonTrajectory)
    
    setTrajectory0([0.0, 1.5, 1.5], 0.5, 90.0)
    setTrajectory1([1.5, 1.5, 1.5], 0.5, 0.0)
