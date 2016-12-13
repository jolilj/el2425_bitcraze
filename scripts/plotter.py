#!/usr/bin/env python
import rospy
import math
from anchorpos import get_anchors_pos
from visualization_msgs.msg import Marker, MarkerArray
from el2425_bitcraze.srv import SetTargetPosition 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray as Array
from geometry_msgs.msg import Point


#======== PLOTTER CLASS ===============
# This class handles trajectory plotting in rviz by subscribing to the following topics
#   * goal - crazyflies PID reference
#   * target_pos - the user specified target position
#   * crazyflie_position - the estimated position of the crazyflie
# and publish the coresponding markers to rviz.

class Plotter:
    def __init__(self, x, y, z):
        
        # Each plotted marker needs a unique id, 0 and 1 reserved for crazyflies        
        self.markerId = 2
        self.namespace = rospy.get_namespace()

        # Define colors
        if(self.namespace == "/crazyflie0/" or self.namespace == "/crazyflie"):
            self.cfId = 0
            self.cfMarkerColor = [169.0/255, 248.0/255, 105.0/255]
            self.cfTrajColor = self.cfMarkerColor
            self.trajColor =  [71.0/255, 150.0/255, 0.0/255]
            self.targetColor = self.trajColor
        else:
            self.cfId = 1
            self.cfMarkerColor = [221.0/255, 166.0/255, 209.0/255]
            self.cfTrajColor = self.cfMarkerColor
            self.trajColor = [166.0/255, 1.0/255, 145.0/255]
            self.targetColor = self.trajColor

        self.anchorColor = [1, 0, 0]
        
        # Plot is based on time step length, needs to initialize previous time step and step length
        self.prevTime = rospy.get_time()
        self.posUpdateTimeStep = 0.2

        # It takes some time for the extended kalman filter used for cf position estimation to converge.
        # The cf trajectory should therefore not be plotted until the first user specified target point is published
        self.shouldPlotCF = False
        
        # Subscribe to topics in relative namespace (that is for a specific crazyflie)
        rospy.Subscriber('goal', PoseStamped, self.goalCallback)
        rospy.Subscriber('target_pos', Array, self.targetPosCallback)
        rospy.Subscriber('crazyflie_position', Point, self.cfPositionCallback)

        # Publish on global rviz topics
        self.targetPosPub = rospy.Publisher("/viz" + self.namespace + "target_points", MarkerArray, queue_size=1, latch=True)
        self.trajPub = rospy.Publisher("/viz" + self.namespace + "trajectory", MarkerArray, queue_size=10, latch=True)
        self.cfTrajPub = rospy.Publisher("/viz" + self.namespace + "cf_trajectory", MarkerArray, queue_size=10, latch=True)
        self.cfPosPub = rospy.Publisher("/viz" + self.namespace + "cf_pos", Marker, queue_size=10, latch=True)
        self.anchorPub = rospy.Publisher("/viz/anchor_pos", MarkerArray, queue_size=10, latch=True)

        # Variables storing trajectories 
        self.trajectory = MarkerArray()
        self.goalMarkers = MarkerArray()
        self.cfTrajectory = MarkerArray()
    
        # Plot initial position
        self.plotTrajectory(x,y,z)
        
        # Plot anchor positions
        self.plotAnchors(self.getAnchorsPos())
    
    # ======= TOPIC CALLBACKS ================================================== 
    def cfPositionCallback(self, point):
        # Plot trajectory
        time = rospy.get_time()
        if self.shouldPlotCF and (time - self.prevTime) > self.posUpdateTimeStep:
            self.prevTime = time
            self.plotCFTrajectory(point.x, point.y, point.z)
    
        # Plot cf position
        self.plotCFPosition(point.x, point.y, point.z)
            

    def goalCallback(self, goal):
        xp = self.trajectory.markers[-1].pose.position.x
        yp = self.trajectory.markers[-1].pose.position.y
        zp = self.trajectory.markers[-1].pose.position.z

        xn = goal.pose.position.x
        yn = goal.pose.position.y
        zn = goal.pose.position.z
        
        # Only plot reference trajectory if it changed from previous time step
        if not (xp == xn and yp == yn and zp == zn):
            self.plotTrajectory(xn, yn, zn)

    def targetPosCallback(self, msg):
        # When first target is published we can assume the ekf filter has converged
        # and hence start plotting the crazyflie trajectory
        if not self.shouldPlotCF:
            self.shouldPlotCF = True    

        self.plotTargetPositions(msg.data[0], msg.data[1], msg.data[2])

    # ====================================================================

    # =========== PUBLISH MARKERS ========================================
    def plotCFPosition(self, x,y,z):
        self.markerId = self.markerId + 1
        marker = self.initMarker(x, y, z)
        marker.id = self.cfId
        marker.color.r = self.cfTrajColor[0]
        marker.color.g = self.cfTrajColor[1] 
        marker.color.b = self.cfTrajColor[2]
        marker.color.a = 1

        self.cfPosPub.publish(marker)

    def plotTargetPositions(self,x,y,z):
        self.markerId = self.markerId + 1
        marker = self.initMarker(x, y, z)
        marker.id = self.markerId
        marker.color.r = self.targetColor[0]
        marker.color.g = self.targetColor[1]
        marker.color.b = self.targetColor[2]
        marker.color.a = 1

        self.goalMarkers.markers.append(marker)
        self.targetPosPub.publish(self.goalMarkers)

    def plotCFTrajectory(self, x, y, z):
        self.markerId = self.markerId + 1
        marker = self.initMarker(x, y, z, typee=2, scale=0.05)
        marker.id = self.markerId
        marker.color.r = self.cfTrajColor[0]
        marker.color.g = self.cfTrajColor[1] 
        marker.color.b = self.cfTrajColor[2]
        marker.color.a = 1

        self.cfTrajectory.markers.append(marker)
        self.cfTrajPub.publish(self.cfTrajectory)

    def plotTrajectory(self,x,y,z):
        self.markerId = self.markerId + 1
        marker = self.initMarker(x, y, z, typee=2, scale=0.05)
        marker.id = self.markerId 
        marker.color.r = self.trajColor[0]
        marker.color.g = self.trajColor[1]
        marker.color.b = self.trajColor[2]
        marker.color.a = 1
        self.trajectory.markers.append(marker) 
        self.trajPub.publish(self.trajectory)

    def plotAnchors(self, anchorPos):
        markerArray = MarkerArray()
        for i in anchorPos:
            self.markerId = self.markerId + 1
            marker = self.initMarker(anchorPos[i][0], anchorPos[i][1], anchorPos[i][2])
            marker.id = self.markerId 
            marker.color.r = self.anchorColor[0]
            marker.color.g = self.anchorColor[1]
            marker.color.b = self.anchorColor[2]
            marker.color.a = 1
            markerArray.markers.append(marker) 

        self.anchorPub.publish(markerArray)

    #======================================================================
    # Helper functions
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

    def getAnchorsPos(self):
        n = rospy.get_param("n_anchors")
        anchorsPos = {}

        for i in range(n):
            anchorsPos[i] = rospy.get_param("anchor{}_pos".format(i))

        return anchorsPos

# ============= MAIN LOOP ===================================================
if __name__ == "__main__":
    rospy.init_node("plotter")
    x = rospy.get_param("x")
    y = rospy.get_param("y")
    z = rospy.get_param("z")
    sim = Plotter(x,y,z)
    rospy.spin()
