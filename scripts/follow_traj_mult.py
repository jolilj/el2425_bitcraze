#!/usr/bin/env python
import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

global type_of_traj0 
global type_of_traj1
class Trajectory:
    def __init__(self, x0, y0, z0, x1, y1, z1):

        self.rate = rospy.Rate(5) # 5hz     

        # Set the initial points of the trajectory to the initial hover points 
        # of respective crazyflies
        self.x0_init = x0;
        self.y0_init = y0;
        self.z0_init = z0;

        self.x1_init = x1;
        self.y1_init = y1;
        self.z1_init = z1;

        # Call the Target position service for individual crazyflie
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie0/set_target_position', SetTargetPosition)
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie1/set_target_position', SetTargetPosition)

        # initialize the center points variables for both
        self.x0_cent = 0
        self.y0_cent = 0
        self.z0_cent = 0

        self.x1_cent = 0
        self.y1_cent = 0
        self.z1_cent = 0

        self.theta = 0

        self.x0_new = 0
        self.y0_new = 0
        self.z0_new = 0

        self.x1_new = 0
        self.y1_new = 0
        self.z1_new = 0

    def FlyMultipleInCircle(self,r0, r1):


        self.theta = 0
        theta_rad = 0

        self.CalculateCenterPoints(r0, r1, theta_rad)
        
        while not rospy.is_shutdown():
            theta_rad = math.pi*self.theta/180
            self.CalculateWayPoints(r0, r1 , theta_rad)
            self.callTargetPositionService(self.x0_new, self.y0_new, self.z0_new)
            self.callTargetPositionService(self.x1_new, self.y1_new, self.z1_new)

            self.theta = self.theta + 5       #10
            if self.theta >= 360:
                self.theta = 0
            self.rate.sleep()

    def CalculateCenterPoints(self, r0, r1 , angle):

        self.x0_cent = self.x0_init - r0*math.cos(angle)
        if (type_of_traj0 == 'h'):
            self.y0_cent = self.y0_init - r0*math.sin(angle)
            self.z0_cent = self.z0_init
        if (type_of_traj0 == 'v'):
            self.y0_cent = self.y0_init
            self.z0_cent = self.z0_init - r0*math.sin(angle)
        
        self.x1_cent = self.x1_init + r1*math.cos(angle)
        if (type_of_traj1 == 'h'):
            self.y1_cent = self.y1_init + r1*math.sin(angle)
            self.z1_cent = self.z1_init
        if (type_of_traj1 == 'v'):
            self.y1_cent = self.y1_init
            self.z1_cent = self.z1_init + r1*math.sin(angle)

    def CalculateWayPoints(self, r0, r1 , angle):
        
        self.x0_new = self.x0_cent + r0*math.cos(angle)
        if (type_of_traj0 == 'h'):
            self.y0_new = self.y0_cent + r0*math.sin(angle)
            self.z0_new = self.z0_cent
        if (type_of_traj0 == 'v'):
            self.y0_new = self.y0_cent
            self.z0_new = self.z0_cent + r0*math.sin(angle)

        self.x1_new = self.x1_cent - r1*math.cos(angle)
        if (type_of_traj1 == 'h'):
            self.y1_new = self.y1_cent - r1*math.sin(angle)
            self.z1_new = self.z1_cent
        if (type_of_traj1 == 'v'):
            self.y1_new = self.y1_cent
            self.z1_new = self.z1_cent - r1*math.sin(angle)

if __name__ == "__main__":
    rospy.init_node('Trajectory')

    # read the parameters from the launch file
    # these points are initial hover points which the user specified while launching the file

    # Initial starting points for crazy flie 0
    x0 = rospy.get_param("/crazyflie0/x")
    y0 = rospy.get_param("/crazyflie0/y")
    z0 = rospy.get_param("/crazyflie0/z")

    # Initial starting points for crazy flie 1
    x1 = rospy.get_param("/crazyflie1/x")
    y1 = rospy.get_param("/crazyflie1/y")
    z1 = rospy.get_param("/crazyflie1/z")

    traj = Trajectory(x0, y0, z0, x1, y1, z1)

    # read the arguments which user specifies while running the execultable file of this script

    type_of_traj0 = sys.argv[1]
    type_of_traj1 = sys.argv[2]

    radius0 = float(sys.argv[3])
    radius1 = float(sys.argv[4])

    traj.FlyMultipleInCircle(radius0,radius1)
    