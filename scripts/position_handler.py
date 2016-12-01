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

	self.trajectory = [[],[],[]]

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

    def generateLinearTrajectory(self, x, y, z):
	x_old = self.msg.pose.position.x
	y_old = self.msg.pose.position.y
	z_old = self.msg.pose.position.z
	
	if x == x_old :
	    kx = 0
	else: 
	    Tx = 5*(x - x_old)
	    kx = (x - x_old)/Tx
	if y == y_old :
	    ky = 0
	else:
	    Ty = 5*(y - y_old)
	    ky = (y - y_old)/Ty
	if z == z_old :
	    kz = 0
	else:
	    Tz = 5*(z - z_old)
	    kz = (z - z_old)/Tz

	for i in range(1, Tx+1):
	    self.trajectory.x.append([x_old + kx*i])
	for i in range(1, Ty+1):
	    self.trajectory.y.append([y_old + ky*i])
	for i in range(1, Tz+1):
	    self.trajectory.z.append([z_old + kz*i])
	
	

    def goalServiceCallback(self, req):
        print "Msg received: [x: %f, y: %f, z: %f]\n" %(req.x, req.y, req.z)
        self.generateLinearTrajectory(req.x, req.y, req.z)
        
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
