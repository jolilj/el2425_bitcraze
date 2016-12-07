#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
#from el2425_bitcraze.srv import SetGoal 
from el2425_bitcraze.srv import *
#from rospy_tutorials.msg import Floats
  

def way_points(r):
    rospy.init_node('way_points')
    theta = 0
    x_cent = 0
    y_cent = 0
    z_cent = 1
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        theta_rad = math.pi*theta/180
        x_new = r*math.cos(theta_rad)
        y_new = r*math.cos(theta_rad)
        z_new = z_cent
        theta = theta + 10
        print "Theta: %f" %(theta)
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #rospy.publish(hello_str)
        rate.sleep()
        if theta >= 360:
            theta = 0

if __name__ == "__main__":
    try:  
        radius = int(sys.argv[1])
        way_points(radius)
    except rospy.ROSInterruptException:
        pass

