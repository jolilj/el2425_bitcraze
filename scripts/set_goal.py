#!/usr/bin/env python

import rospy
import sys
import tf
from el2425_bitcraze.srv import SetGoal 
from geometry_msgs.msg import PoseStamped

class PositionHandler:
    def __init__(self, x, y, z):
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        yaw = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.msg.pose.orientation.x = quaternion[0]
        self.msg.pose.orientation.y = quaternion[1]
        self.msg.pose.orientation.z = quaternion[2]
        self.msg.pose.orientation.w = quaternion[3]

        self.rate = rospy.Rate(30)
        self.pub = rospy.Publisher("/crazyflie/goal", PoseStamped, queue_size=1)

        self.setPointService = rospy.Service('/crazyflie/setgoal', SetGoal, self.updateGoal)

    def updateGoal(req):
        print("Msg received:" + req.toString())
        goal = req.goalstring.split()
        self.msg.pose.position.x = float(goal[0])
        self.msg.pose.position.y = float(goal[1])
        self.msg.pose.position.z = float(goal[2])

    def run():
        self.msg.header.seq += 1
        self.msg.header.stamp = rospy.Time.now()
        pub.publish(self.msg)
        rate.sleep();

        
        

if __name__ == "__main__":
    rospy.init_node('position_handler', anonymous=True)
    setPoint = []
    if len(sys.argv) > 1:
        setPoint = sys.argv[1].split()
    else:
        setPoint = raw_input('Please specify set point as "x y z"');     
    
    x = float(goal[0]);
    y = float(goal[1]);
    z = float(goal[2]);

    posHandler = PositionHandler(x,y,z)
    posHandler.run()
