#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class Trajectory:
    def __init__(self):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.point = Point()
        self.rate = rospy.Rate(6) # 3hz     
        self.x_init = 0
        self.y_init = 0
        self.z_init = 0
        # Subscribe to the topics
        rospy.Subscriber('/crazyflie/crazyflie_position', Point, self.positionCallback)   # Position of crazyfile
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie/set_target_position', SetTargetPosition)

 
    def way_points_circle(self,r):


        self.x_init = 1
        self.y_init = 1.6
        self.z_init = 1.5
        theta = 0
        theta_rad = 0
        
        print "Point X: %f" %self.x_init

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
            #print "Point: %f" %(self.point.x)
            #rate.sleep()
            if theta >= 360:
                theta = 0
            self.rate.sleep()

    def way_points_line(self,x,y,z):
        self.callTargetPositionService(x, y, z)
        self.x_init = x
        self.y_init = y
        self.z_init = z

        print "Point X: %f" %self.x_init

# ======= TOPIC CALLBACKS ================================================== 

    def positionCallback(self, point):
        self.point = point

if __name__ == "__main__":
    rospy.init_node('Trajectory')
    traj = Trajectory();
    type_of_traj = (sys.argv[1])
    if type_of_traj == 'c':
        radius = float(sys.argv[2])
        traj.way_points_circle(radius)
    if type_of_traj == 'l':
        x_line = float(sys.argv[2])
        y_line = float(sys.argv[3])
        z_line = float(sys.argv[3])
        traj.way_points_line(x_line,y_line,z_line)
