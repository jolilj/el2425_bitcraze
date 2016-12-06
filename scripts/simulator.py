#!/usr/bin/env python
import rospy
import math
from anchorpos import get_anchors_pos
from visualization_msgs.msg import Marker, MarkerArray
from el2425_bitcraze.srv import SetGoal 
from geometry_msgs.msg import PoseStamped

class Simulator:
    def __init__(self, x, y, z):
        rospy.Subscriber("/crazyflie/goal", PoseStamped, self.updatePosition)
        self.setGoalService = rospy.Service('/crazyflie/setgoal', SetGoal, self.goalServiceCallback)
        self.markerPub = rospy.Publisher("markers", MarkerArray, queue_size=1)
        self.trajPub = rospy.Publisher("trajectory", MarkerArray, queue_size=1)
        
        self.trajectory = MarkerArray()
        self.trajectory.markers.append(initMarker(x,y,z)) 

        self.goalMarkers = MarkerArray()
        
        self.trajPub.publish(self.trajectory)

    def updatePosition(self, goal):
        self.trajectory.marker.append(initMarker(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)) 
        self.trajPub.publish(self.trajectory)

    def goalServiceCallback(self, req):
        self.goalMarkers.markers.append(initMarker(req.x, req.y, req.z, [0, 0, 1]))
        self.markerPub.publish(self.goalMarkers)

    def initMarker(x,y,z,rgb=[0, 1, 0]):
        marker = Marker()

        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()

        marker.pose.position.x = x 
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 1

        marker.lifetime = rospy.Duration()

        return marker

if __name__ == "__main__":
    rospy.init_node("simulator")
    sim = Simulator(x,y,z)
    rospy.spin()
