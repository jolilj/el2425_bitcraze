#!/usr/bin/env python

import rospy
import math
import sys
import tf
from el2425_bitcraze.srv import SetTargetPosition
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray as Array

# ======= PositionHandler class ==================================
# Calculates reference points for the crazyflie PID controller based on target points set by the user
# Defines the service set_target_position which when called executes an interpolator that sequentally 
# updates the goal topic (which is the topis that the crazyflies PID use as reference
# Has the following tweak parameters
#   * deltaT - Time between published goal points (or reference points if you will) This has to be long enough so the PID
#     manages to control the crazyflie to that point before publishing a new point
#   * k - The distance to step in the direction of the target point from the current goal position,
#         this has to be small enough so that we don't get the large acceleration change due to a large error in the PID controller

# ==== TWEAK PARAMS ====
deltaT = 0.2    # 0.2
vel = 0.5
class PositionHandler:
    def __init__(self):
        self.isSingleCF = 0
        namespace = rospy.get_namespace()
        if namespace == "/crazyflie/":
            self.isSingleCF = 1
        elif namespace == "/crazyflie0/":
            otherNamespace = "/crazyflie1/"
        else:
            otherNamespace = "/crazyflie0/"

        x = rospy.get_param("x")
        y = rospy.get_param("y")
        z = rospy.get_param("z")


        # The msg that will be published on the goal topic
        self.referenceMsg = PoseStamped()
        self.referenceMsg.header.seq = 0
        self.referenceMsg.header.stamp = rospy.Time.now()
        
        self.referenceMsg.pose.position.x = x
        self.referenceMsg.pose.position.y = y
        self.referenceMsg.pose.position.z = z
        
        self.targetPosition = [x,y,z]
        self.deltaT = deltaT   # 0.2

        yaw = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.referenceMsg.pose.orientation.x = quaternion[0]
        self.referenceMsg.pose.orientation.y = quaternion[1]
        self.referenceMsg.pose.orientation.z = quaternion[2]
        self.referenceMsg.pose.orientation.w = quaternion[3]

        self.rate = rospy.Rate(20)

        self.goalPub = rospy.Publisher("goal", PoseStamped, queue_size=1)
        self.targetPosPub = rospy.Publisher("target_pos", Array, queue_size=1, latch=True)

        rospy.Service('set_target_position', SetTargetPosition, self.targetPositionServiceCallback)
        
        self.isRunning = False

        if not self.isSingleCF:
            x2 = rospy.get_param(otherNamespace + "x")
            y2 = rospy.get_param(otherNamespace + "y")
            z2 = rospy.get_param(otherNamespace + "z")

            self.cf2Pos = [x2, y2, z2]
            self.cf2PrevPos = [x2, y2, z2]

            rospy.Subscriber(otherNamespace + "goal", PoseStamped, self.cf2GoalCallback)
   
    # ====== Target Position Service Callback ==== 
    # When the target_position service is called we want to
    #     * publish the new target position on the target_pos topic
    #       This is mainly for logging and plotting.
    #     * update the internal target position variable used when calculating the intermediate goals 
    def targetPositionServiceCallback(self, req):
        self.targetPosition = [req.x, req.y, req.z]
        msg = Array()
        msg.data = self.targetPosition
        self.targetPosPub.publish(msg)
        return()
    
    # ===== Interpolator ====
    # Based on previous goal position this function publishes a new goal incremented in the direction towards the target position
    # The incrementation is determined by the parameter k. 
    # The second crayflies position and direction is also passed into the collision avoidance algorithm

    def calculatePIDReferenceGoal(self):
        x_old = self.referenceMsg.pose.position.x
        y_old = self.referenceMsg.pose.position.y
        z_old = self.referenceMsg.pose.position.z

        cfDirection = self.calcDirection(self.targetPosition,[x_old, y_old, z_old])

        k = vel*self.deltaT

        nextPos = [x_old+k*cfDirection[0], y_old+k*cfDirection[1],x_old+k*cfDirection[2]]
        
        if not self.isSingleCF:
            cf2Direction = self.calcDirection(self.cf2Pos, self.cf2PrevPos)
            cf2NextPos = [self.cf2Pos[0]+k*cf2Direction[0], self.cf2Pos[1]+k*cf2Direction[1],self.cf2Pos[2]+k*cf2Direction[2]]
            #nextPos = collisionAvoidance(nextPos, cfDirection, cf2NextPos, cf2Direction)
            self.cf2PrevPos = self.cf2Pos

        #If overshoot set new position to goal
        if math.fabs(self.targetPosition[0]-x_old) <= math.fabs(k*cfDirection[0]):
            x_new = self.targetPosition[0]
            y_new = self.targetPosition[1]
            z_new = self.targetPosition[2]
            
        self.referenceMsg.pose.position.x = x_new
        self.referenceMsg.pose.position.y = y_new
        self.referenceMsg.pose.position.z = z_new
        

    # ====== Callback for second crazyflie goal =====
    # ===============================================

    def cf2GoalCallback(self,goal):
        self.cf2Pos = [goal.pose.position.x, goal.pose.position.y, goal.pose.position.z]

    def calcDirection(self, pos1, pos2):
        delta_x = float(pos1[0]-pos2[0])
        delta_y = float(pos1[1]-pos2[1])
        delta_z = float(pos1[2]-pos2[2])
        
        delta_norm = (math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2)+math.pow(delta_z,2)))
        if not delta_norm == 0:
            return [delta_x/delta_norm, delta_y/delta_norm, delta_z/delta_norm]
        return [0, 0, 0]
        


    # ====== RUN LOOP =======
    # This is the main loop, notice that the PID reference signal (published on the goal topic) is calculated every deltaT second
    def run(self):
        prevTime = rospy.get_time()
        while not rospy.is_shutdown():
            time = rospy.get_time()
            if time-prevTime > self.deltaT:
                self.calculatePIDReferenceGoal()
                prevTime = time

            self.referenceMsg.header.seq += 1
            self.referenceMsg.header.stamp = rospy.Time.now()
            self.goalPub.publish(self.referenceMsg)
            self.rate.sleep();

if __name__ == "__main__":
    rospy.init_node("position_handler")

    posHandler = PositionHandler()
    posHandler.run()
