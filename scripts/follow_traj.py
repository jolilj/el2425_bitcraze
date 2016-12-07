#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class Trajectory:
    def __init__(self, radius):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.point = Point()
        self.rate = rospy.Rate(1) # 100hz     
        # Subscribe to the topics
        rospy.Subscriber('/crazyflie/crazyflie_position', Point, self.positionCallback)   # Position of crazyfile
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie/set_target_position', SetTargetPosition)

# ======= TOPIC CALLBACKS ================================================== 
 
    def way_points(self,r):
        x_old = -0.5
        y_old = 1.8
        z_old = 1

        theta = 0
        theta_rad = 0
        x_cent = x_old - r*math.cos(theta_rad)
        y_cent = y_old - r*math.sin(theta_rad)
        z_cent = z_old
        self.flag = 0
        while not rospy.is_shutdown():
            theta_rad = math.pi*theta/180
            x_new = x_cent + r*math.cos(theta_rad)
            y_new = y_cent + r*math.sin(theta_rad)
            z_new = z_cent
            self.callTargetPositionService(x_new, y_new, z_new)
            theta = theta + 20
            print "Point: %f" %(self.point.x)
            #rate.sleep()
            if theta >= 360:
                theta = 0
            self.rate.sleep()
            if not self.flag:
                self.rate.sleep()
                self.rate.sleep()
                self.flag = 1



    def positionCallback(self, point):
        self.point = point

if __name__ == "__main__":
    rospy.init_node('Trajectory')
    radius = float(sys.argv[1])
    traj_circle = Trajectory(radius);
    traj_circle.way_points(radius)
