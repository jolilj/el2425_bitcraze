#!/usr/bin/env python
import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class Trajectory:
    def __init__(self, x0, y0, z0, x1, y1, z1):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()

        self.rate = rospy.Rate(5) # 5hz     

        self.x0_init = x0;
        self.y0_init = y0;
        self.z0_init = z0;

        self.x1_init = x1;
        self.y1_init = y1;
        self.z1_init = z1;

        # Subscribe to the topics
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie0/set_target_position', SetTargetPosition)
        self.callTargetPositionService = rospy.ServiceProxy('/crazyflie1/set_target_position', SetTargetPosition)


    def way_points_multiple_circle(self,r0, r1):


        theta = 0
        theta_rad = 0
        
        x0_cent = self.x0_init - r0*math.cos(theta_rad)
        y0_cent = self.y0_init - r0*math.sin(theta_rad)
        z0_cent = self.z0_init

        x1_cent = self.x1_init - r1*math.cos(theta_rad)
        y1_cent = self.y1_init - r1*math.sin(theta_rad)
        z1_cent = self.z1_init

        while not rospy.is_shutdown():
            theta_rad = math.pi*theta/180
            x0_new = x0_cent + r0*math.cos(theta_rad)
            y0_new = y0_cent + r0*math.sin(theta_rad)
            z0_new = z0_cent
            self.callTargetPositionService(x0_new, y0_new, z0_new)

            x1_new = x1_cent + r1*math.cos(theta_rad)
            y1_new = y1_cent + r1*math.sin(theta_rad)
            z1_new = z1_cent
            self.callTargetPositionService(x1_new, y1_new, z1_new)

            theta = theta + 5       #10
            #rate.sleep()
            if theta >= 360:
                theta = 0
            self.rate.sleep()

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

    radius0 = float(sys.argv[1])
    radius1 = float(sys.argv[2])
    traj.way_points_multiple_circle(radius0,radius1)
    