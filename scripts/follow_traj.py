#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class Trajectory:
    def __init__(self, x, y, z):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.point = Point()
        self.rate = rospy.Rate(5) # 5hz     

        self.x_init = x;
        self.y_init = y;
        self.z_init = z;

        # Subscribe to the topics
        rospy.Subscriber('/crazyflie/crazyflie_position', Point, self.positionCallback)   # Position of crazyfile
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie/set_target_position', SetTargetPosition)

# ======= TOPIC CALLBACKS ================================================== 

    def positionCallback(self, point):
        self.point = point

    def way_points_circle(self,r):


        theta = 0
        theta_rad = 0
        
        x_cent = self.x_init - r*math.cos(theta_rad)
        y_cent = self.y_init - r*math.sin(theta_rad)
        z_cent = self.z_init

        while not rospy.is_shutdown():
            theta_rad = math.pi*theta/180
            x_new = x_cent + r*math.cos(theta_rad)
            y_new = y_cent + r*math.sin(theta_rad)
            z_new = z_cent
            self.callTargetPositionService(x_new, y_new, z_new)

            theta = theta + 5       #10
            #rate.sleep()
            if theta >= 360:
                theta = 0
            self.rate.sleep()

    def way_points_line(self,x,y,z):
        self.callTargetPositionService(x, y, z)

        print "Point X: %f" %self.x_init

if __name__ == "__main__":
    rospy.init_node('Trajectory')

    x = rospy.get_param("/crazyflie/x")
    y = rospy.get_param("/crazyflie/y")
    z = rospy.get_param("/crazyflie/z")

    print "Point X: %f" %x
    print "Point Y: %f" %y
    print "Point Z: %f" %z

    traj = Trajectory(x,y,z)

    type_of_traj = (sys.argv[1])

    if type_of_traj == 'c':
        radius = float(sys.argv[2])
        traj.way_points_circle(radius)
    if type_of_traj == 'l':
        x_line = float(sys.argv[2])
        y_line = float(sys.argv[3])
        z_line = float(sys.argv[4])
        traj.way_points_line(x_line,y_line,z_line)
