#!/usr/bin/env python

import rospy
import sys
import tf
from el2425_bitcraze.srv import SetGoal 
from geometry_msgs.msg import PoseStamped

class PositionHandler:
    def __init__(self):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        
        self.msg.pose.position.x = 0
        self.msg.pose.position.y = 0
        self.msg.pose.position.z = 0.1

        yaw = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher("/crazyflie/goal", PoseStamped, queue_size=1)
        self.setGoalService = rospy.Service('/crazyflie/setgoal', SetGoal, self.updateGoal)
        self.isRunning = False

    def updateGoal(self, req):
        print "Msg received: [x: %f, y: %f, z: %f]\n" %(req.x, req.y, req.z)
        self.msg.pose.position.x = req.x
        self.msg.pose.position.y = req.y
        self.msg.pose.position.z = req.z
        
        return()

    def run(self):
        while not rospy.is_shutdown():
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            self.rate.sleep();

if __name__ == "__main__":
    rospy.init_node("position_handler")
    posHandler = PositionHandler()
    posHandler.run()
