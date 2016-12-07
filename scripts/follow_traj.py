#!/usr/bin/env python

import rospy
import math
import sys
import tf
#from el2425_bitcraze.srv import SetGoal 
from el2425_bitcraze.srv import *
from geometry_msgs.msg import PoseStamped

class Trajectory:
    def __init__(self, radius):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        
        #self.msg.pose.position.x = x
        #self.msg.pose.position.y = y
        #self.msg.pose.position.z = z

        #self.goal = [x,y,z]
        #self.delT = 0.5

        yaw = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

        self.rate = rospy.Rate(30)
        #self.pub = rospy.Publisher("/crazyflie/goal", PoseStamped, queue_size=1)
        #self.setGoalService = rospy.Service('/crazyflie/setgoal', SetGoal, self.goalServiceCallback)
        #self.isRunning = False
 
    def way_points(self,r):
        x_old = self.msg.pose.orientation.x
        y_old = self.msg.pose.orientation.y
        z_old = self.msg.pose.orientation.z
        theta = 0
        x_cent = x_old - r*math.cos(theta_rad)
        y_cent = y_old - r*math.sin(theta_rad)
        z_cent = z_old
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            theta_rad = math.pi*theta/180
            x_new = x_cent + r*math.cos(theta_rad)
            y_new = y_cent + r*math.sin(theta_rad)
            z_new = z_cent
            theta = theta + 10
            print "Theta: %f" %(theta)
            #rate.sleep()
            if theta >= 360:
                theta = 0

if __name__ == "__main__":
    rospy.init_node('Trajectory')
    radius = int(sys.argv[1])
    traj_circle = Trajectory(radius);
    traj_circle.way_points(radius)
