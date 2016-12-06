#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetGoal 
from geometry_msgs.msg import PoseStamped

class PositionHandler:
    def __init__(self, x, y, z):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        
        self.msg.pose.position.x = x
        self.msg.pose.position.y = y
        self.msg.pose.position.z = z

        self.goal = [x,y,z]
        self.delT = 0.5

        yaw = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher("/crazyflie/goal", PoseStamped, queue_size=1)
        self.setGoalService = rospy.Service('/crazyflie/setgoal', SetGoal, self.goalServiceCallback)
        self.isRunning = False

    def calcIntermediateGoal(self):
        x_old = self.msg.pose.position.x
        y_old = self.msg.pose.position.y
        z_old = self.msg.pose.position.z

        delta_x = self.goal[0]-x_old
        delta_y = self.goal[1]-y_old
        delta_z = self.goal[2]-z_old

        print "delta x: %f" %(delta_x)
        
        delta_len = (math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2)+math.pow(delta_z,2)))
        print "delta_len: %f" %(delta_len)
        k = 0
        if not delta_len == 0:
            k = 0.4*self.delT/delta_len

        x_new = x_old+k*delta_x
        y_new = y_old+k*delta_y
        z_new = z_old+k*delta_z
	  
        #If overshoot set new position to goal
        if math.fabs(self.goal[0]-x_old) < math.fabs(k*delta_x):
            print("Hej")
            x_new = self.goal[0]
            y_new = self.goal[1]
            z_new = self.goal[2]
            
        self.msg.pose.position.x = x_new
        self.msg.pose.position.y = y_new
        self.msg.pose.position.z = z_new
        
        print "Intermediate goal: [x: %f, y: %f, z: %f]\n" %(x_new, y_new, z_new)
        

    def goalServiceCallback(self, req):
        print "Msg received: [x: %f, y: %f, z: %f]\n" %(req.x, req.y, req.z)
        self.goal = [req.x, req.y, req.z]
        return()

    def run(self):
        prevTime = rospy.get_time()
        while not rospy.is_shutdown():
            time = rospy.get_time()
            if time-prevTime > self.delT:
                print "Delta T: %f" %(time-prevTime)
                self.calcIntermediateGoal()
                prevTime = time

            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            self.rate.sleep();

if __name__ == "__main__":
    rospy.init_node("position_handler")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    posHandler = PositionHandler(x,y,z)
    posHandler.run()
