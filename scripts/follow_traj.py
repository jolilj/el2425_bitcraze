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

        self.rate = rospy.Rate(5) # 3hz     

        self.x_init = x;
        self.y_init = y;
        self.z_init = z;

        # Call the Target position service
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie/set_target_position', SetTargetPosition)

    def way_points_horiz_circle(self,r):


        theta = 0
        theta_rad = 0

        # calculate the center point of the circle 
        # we are assuming that the initial hover points are on the circumference of the cirlce
        
        x_cent = self.x_init - r*math.cos(theta_rad)
        y_cent = self.y_init - r*math.sin(theta_rad)
        z_cent = self.z_init

        while not rospy.is_shutdown():
            theta_rad = math.pi*theta/180
            x_new = x_cent + r*math.cos(theta_rad)
            y_new = y_cent + r*math.sin(theta_rad)
            z_new = z_cent
    
            # send the new way points to the service SetTargetPosition
            self.callTargetPositionService(x_new, y_new, z_new)

            # step/ increment of 5 degrees.
            # this step decides the shape of the cirlce
            # if we change it to 90 degrees the tarjectory will become square
            theta = theta + 5       #10
            #print "Point: %f" %(self.point.x)

            if theta >= 360:
                theta = 0
            self.rate.sleep()

    def way_points_vert_circle(self,r):

        theta = 0
        theta_rad = 0

        # calculate the center point of the circle 
        # we are assuming that the initial hover points are on the circumference of the cirlce
        
        x_cent = self.x_init - r*math.cos(theta_rad)
        z_cent = self.z_init - r*math.sin(theta_rad)
        y_cent = self.y_init

        while not rospy.is_shutdown():
            theta_rad = math.pi*theta/180
            x_new = x_cent + r*math.cos(theta_rad)
            z_new = z_cent + r*math.sin(theta_rad)
            y_new = y_cent
    
            # send the new way points to the service SetTargetPosition
            self.callTargetPositionService(x_new, y_new, z_new)

            # step/ increment of 5 degrees.
            # this step decides the shape of the cirlce
            # if we change it to 90 degrees the tarjectory will become square
            theta = theta + 5       #10
            #print "Point: %f" %(self.point.x)

            if theta >= 360:
                theta = 0
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('Trajectory')


    # read the parameters from the launch file
    # these points are initial hover points which the user specified while launching the file

    # Initial starting points for crazy flie 0
    x = rospy.get_param("/crazyflie/x")
    y = rospy.get_param("/crazyflie/y")
    z = rospy.get_param("/crazyflie/z")


    traj = Trajectory(x, y, z)
    # read the first argument which gives the type of the trajectory
    # character 'h' for Horizental 
    # character 'v' for vertical
    type_of_traj = sys.argv[1]
    # read the second argument which is the radius of the circle
    radius = float(sys.argv[2])
    if type_of_traj == 'h':
        traj.way_points_horiz_circle(radius)
    if type_of_traj == 'v':
        traj.way_points_vert_circle(radius)
