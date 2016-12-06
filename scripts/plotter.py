#!/usr/bin/env python
import rospy
import math
from anchorpos import get_anchors_pos
from visualization_msgs.msg import Marker, MarkerArray
from el2425_bitcraze.srv import SetGoal 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray as Array
from geometry_msgs.msg import Point

class Plotter:
    def __init__(self, x, y, z):
        self.idd = 0
        self.prevTime = rospy.get_time()
        self.posUpdateTime = 0.2
        self.gotSecondTarget = 0
        rospy.Subscriber("/crazyflie/goal", PoseStamped, self.goalCallback)
        rospy.Subscriber('/crazyflie/target_pos', Array, self.targetPosCallback)
        rospy.Subscriber('/crazyflie/crazyflie_position', Point, self.cfPositionCallback)

        self.markerPub = rospy.Publisher("/viz/target_points", MarkerArray, queue_size=1, latch=True)
        self.trajPub = rospy.Publisher("/viz/trajectory", MarkerArray, queue_size=10, latch=True)
        self.cfTrajPub = rospy.Publisher("/viz/cf_trajectory", MarkerArray, queue_size=10, latch=True)
        
        self.trajectory = MarkerArray()
        self.goalMarkers = MarkerArray()
        self.cfTrajectory = MarkerArray()

        initialPos = Array()
        initialPos.data = [x, y, z]
        
        self.targetPosCallback(initialPos)
        
    def cfPositionCallback(self, point):
        time = rospy.get_time()
        if (time - self.prevTime) > self.posUpdateTime and self.gotSecondTarget >= 2:
            self.prevTime = time
            self.idd = self.idd + 1
            marker = self.initMarker(point.x, point.y, point.z, typee=2, scale=0.05)
            marker.id = self.idd
            marker.color.r = 0
            marker.color.g = 1 
            marker.color.b = 0
            marker.color.a = 1

            self.cfTrajectory.markers.append(marker)
            self.cfTrajPub.publish(self.cfTrajectory)
            
        

    def goalCallback(self, goal):
        self.idd = self.idd + 1
        if len(self.trajectory.markers) == 0:
            xp = -100000000
            yp = -100000000
            zp = -100000000
        else:
            currentPosition = self.trajectory.markers[-1].pose.position
            xp = currentPosition.x
            yp = currentPosition.y
            zp = currentPosition.z

        xn = goal.pose.position.x
        yn = goal.pose.position.y
        zn = goal.pose.position.z

        if not (xp == xn and yp == yn and zp == zn):
            marker = self.initMarker(xn, yn, zn, typee=2, scale=0.05)
            marker.id = self.idd 
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
            marker.color.a = 1
            self.trajectory.markers.append(marker) 
            self.trajPub.publish(self.trajectory)

    def targetPosCallback(self, msg):
        self.idd = self.idd + 1
        self.gotSecondTarget = self.gotSecondTarget + 1
        marker = self.initMarker(msg.data[0], msg.data[1], msg.data[2])
        marker.id = self.idd
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1

        self.goalMarkers.markers.append(marker)
        self.markerPub.publish(self.goalMarkers)

    def initMarker(self, x,y,z, typee=1, scale=0.1):
        marker = Marker()

        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        
        marker.type=typee
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.pose.position.x = x 
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        marker.lifetime = rospy.Duration()

        return marker

if __name__ == "__main__":
    rospy.init_node("plotter")
    x = rospy.get_param("/crazyflie/position_handler/x")
    y = rospy.get_param("/crazyflie/position_handler/y")
    z = rospy.get_param("/crazyflie/position_handler/z")
    sim = Plotter(x,y,z)
    rospy.spin()
