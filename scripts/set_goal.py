#!/usr/bin/env python

import rospy
import sys
import tf
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node('publish_goal', anonymous=True)
    goal = []
    if len(sys.argv) > 1:
        goal = sys.argv[1].split()
    else:
        goal = raw_input('Please specify goal as "x_g y_g y_g"');     
    
    x = float(goal[0]);
    y = float(goal[1]);
    z = float(goal[2]);

    rate = rospy.Rate(30)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    #msg.header.frame_id = "/world"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    yaw = 0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher("/crazyflie/goal", PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep();
