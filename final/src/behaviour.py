#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
#import analog_gates
from tf.transformations import euler_from_quaternion
from analog_gates import invoke_gate, prevail_gate

class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return "{}|{}".format(self.x , self.y)

    def distanceTo(self, target):
        return np.sqrt(np.square(target.x - self.x) + np.square(target.y- self.y) )

class Velocity:
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular

    def __repr__(self):
        return "lin: {}|ang: {}".format(self.linear, self.angular)

class RoboterType:
    MOUSE = 1
    CAT = 2

class Behaviour:

    def __init__(self, roboterType):
        
        #properties
        self.positionSelf = Position(0.0,0.0)
        self.positionTarget = Position(0.0,0.0)
        self.velocity = Velocity(0.0, 0.0)
        self.orientationSelf = 0.0
        self.orientationTarget = 0.0
        self.last_angle = 0.0


        self.roboterType = roboterType
        self.pub = None

        self.sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        self.sonar_angles = self.sonar_angles / 360.0 * 2 * np.pi
        self.sonar_ranges = np.zeros(len(self.sonar_angles))

        self.maxLinVel = 1.0
        self.maxAngVel = 1.0

        if self.roboterType == RoboterType.CAT:
        #ros callback listener for own information
            rospy.Subscriber("/cat/p3dx_velocity_controller/odom", Odometry, self.velocitySelfCallback)
            rospy.Subscriber("/cat/base_pose_ground_truth", Odometry, self.positionAndOrientationSelfCallback)
            rospy.Subscriber("/cat/sonar", PointCloud, self.sonar_callback)
            
            #ros callback listener for target information
            rospy.Subscriber("/mouse/base_pose_ground_truth", Odometry, self.positionAndOrientationTargetCallback)

            #ros publisher to give control commands
            self.pub = rospy.Publisher("/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

            self.maxLinVel = 0.4
            self.maxAngVel = 0.8
        else:
            #ros callback listener for own information
            rospy.Subscriber("/mouse/p3dx_velocity_controller/odom", Odometry, self.velocitySelfCallback)
            rospy.Subscriber("/mouse/base_pose_ground_truth", Odometry, self.positionAndOrientationSelfCallback)
            rospy.Subscriber("/mouse/sonar", PointCloud, self.sonar_callback)
            
            #ros callback listener for target information
            rospy.Subscriber("/cat/base_pose_ground_truth", Odometry, self.positionAndOrientationTargetCallback)

            #ros publisher to give control commands
            self.pub = rospy.Publisher("/mouse/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

            self.maxLinVel = 0.35
            self.maxAngVel = 1.0


        #init main loop
        while not rospy.is_shutdown():  
            #self.simulatePosition(self.roboterType, self.positionSelf, self.orientationSelf)
            selfPosOr, targetPosOr = self.simulatePositions() 
            
            direction = self.calcBestCombination(selfPosOr, targetPosOr)
            directionTranslations = [Velocity(1, -2), Velocity(1, -1), Velocity(1, 0.0), Velocity(1, 1), Velocity(1, 2.0)]
            toGo = directionTranslations[direction]
            collAvoidanceOutput = self.performCollisionAvoidances(toGo)
            output = Twist()
            output.linear.x = np.interp(collAvoidanceOutput.linear, [-1,1], [-self.maxLinVel, self.maxLinVel])
            output.angular.z = np.interp(collAvoidanceOutput.angular, [-1,1], [-self.maxAngVel, self.maxAngVel])
            print("OutpuColAvoudancet", collAvoidanceOutput.angular)
            self.pub.publish(output)

            
    def performCollisionAvoidances(self, toGo):
        collAvoidance = self.calculate_collision_avoidance()
        distanceToTarget = self.positionSelf.distanceTo(self.positionTarget)
        output = Velocity(0,0)
        output.linear = toGo.linear

        if np.abs(np.min(self.sonar_ranges) - distanceToTarget)<0.1:  #smallest sonar distance inside the range with distance to target
            output.angular = toGo.angular
        else:
            output.angular = prevail_gate(collAvoidance.angular, toGo.angular)
        return output

    """
    selfPostionOrientations: [[Position...], [Orientation...]]
    """
    def calcBestCombination(self, selfPosOrientations, targetPosOrientations):
        distances = []

        for (indexSelf, selfPos) in enumerate(selfPosOrientations[0]):
            selfDistances = []
            for (index, targetPos) in enumerate(targetPosOrientations[0]):
                distance = selfPos.distanceTo(targetPos)
                selfDistances.append(distance)
            
            targetOptimal = None
            if self.roboterType == RoboterType.CAT:
                targetOptimal = np.max(np.array(selfDistances)) 
            else: 
                targetOptimal = np.min(np.array(selfDistances)) 
            distances.append(targetOptimal)
        distancesWithoutColAvoidance = distances
        indexOptimal = None
        if self.roboterType == RoboterType.CAT:
            distances = np.array(distances) #+ self.getWallDistance()
            indexOptimal = np.argmin(np.array(distances))  
        else:
            distances = np.array(distances) ##- self.getWallDistance()
            indexOptimal = np.argmax(np.array(distances)) 

        print(distances-distancesWithoutColAvoidance, indexOptimal)
        return indexOptimal

    def getWallDistance(self):
        distances = [np.min([self.sonar_ranges[0], self.sonar_ranges[1]]), self.sonar_ranges[2],np.min([self.sonar_ranges[3], self.sonar_ranges[4]]), self.sonar_ranges[5], np.min([self.sonar_ranges[6], self.sonar_ranges[7]])]
        distances = -0.7*np.sqrt(np.array(distances)) + 200 #parameter for collision avoidance
        return distances

    """
    Returns the simulated positions in order: self, target
    """
    def simulatePositions(self):
        if self.roboterType == RoboterType.CAT:
            cat = self.simulatePosition(RoboterType.CAT, self.positionSelf, self.orientationSelf)
            mouse = self.simulatePosition(RoboterType.MOUSE, self.positionTarget, self.orientationTarget)
            return cat, mouse
        else: 
            cat = self.simulatePosition(RoboterType.CAT, self.positionTarget, self.orientationTarget)
            mouse = self.simulatePosition(RoboterType.MOUSE, self.positionSelf, self.orientationSelf)
            return mouse, cat

    def simulatePosition(self, roboterType, position, orientation):
        #discretizing all possibiliies into 5 different directions: straight, half-left, left (respective for right) 
        
        #calcualte the new positions
        #deltaX = arcsin w (w = angle) * d (distance in m, assume the distance is smaller when turning)
        #deltaY = arccos w (w = angle) * d

        if roboterType == RoboterType.CAT:
            w = np.pi * 0.5 + np.array([-1.0, -0.6, 0.0, 0.6, 1.0]) + np.repeat(-1*orientation, 5) #world orientation with own variation
            h = np.array([0.7, 0.9, 1.0, 0.9, 0.7])
        else:
            w = np.pi * 0.5 + np.array([-1.0, -0.6, 0.0, 0.6, 1.0]) + np.repeat(-1*orientation, 5) #world orientation with own variation
            h = np.array([0.7, 0.9, 1.0, 0.9, 0.7])

        deltaX = np.sin(w) * h
        deltaY = np.cos(w) * h

        currentX = np.repeat(position.x, 5) #turn first scalar into array with n arguments
        currentY = np.repeat(position.y, 5)

        newPosX = currentX + deltaX
        newPosY = currentY + deltaY
        newPos = []
        for (index, posX) in enumerate(newPosX):
            newPos.append(Position(posX, newPosY[index]))

        #estimate new orientation
        newOrientation = w #could be improved
        return np.array([newPos, newOrientation])
        

    def calculate_collision_avoidance(self):
        for i, self.sonar_range in enumerate(self.sonar_ranges):
            if self.sonar_range == 0.0:
                rospy.logerr('Catched Zero')
                self.sonar_ranges[i] = 1e-12

        threshold = 0.6
        if np.min([self.sonar_ranges[0], self.sonar_ranges[1], self.sonar_ranges[2], self.sonar_ranges[3], self.sonar_ranges[4], self.sonar_ranges[5], self.sonar_ranges[6], self.sonar_ranges[7]]) < threshold:
            
            #threshold on force = distance
            distancesWithThreshold = (1/self.sonar_ranges * -1) - threshold

            weight = np.array([0.2, 0.3, 0.4, 0.8, 0.8, 0.4, 0.3, 0.2])

            b = np.sin(self.sonar_angles) * distancesWithThreshold * weight
            a = np.cos(self.sonar_angles) * distancesWithThreshold * weight

            F_x = np.sum(b)
            F_y = np.sum(a)

            rotationAngle = np.arctan(F_y / F_x)
            
            if -rotationAngle == self.last_angle:
                rotationAngle = self.last_angle


            #linearForce = 1/((self.sonar_ranges[3] + self.sonar_ranges[4]) / 2) 
            
            #if linearForce < 0.1:
            #    linearForce = 0.1
            linearForce = np.clip(F_x / 20, 0, 1)
        else:
            linearForce = 0
            rotationAngle = 0
        
        self.last_angle = rotationAngle

        velocity_adjustment = Velocity(0.0, 0.0)
        velocity_adjustment.linear  = 1-linearForce
        velocity_adjustment.angular = rotationAngle

        return velocity_adjustment



    def velocitySelfCallback(self, current_odometry):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z

    def positionAndOrientationSelfCallback(self, pos):
        #calcualte orientation from pose information
        fake_odom=pos.pose.pose
        euler=euler_from_quaternion([fake_odom.orientation.x,fake_odom.orientation.y,fake_odom.orientation.z,fake_odom.orientation.w])
        orientation =euler[2]

        self.orientationSelf = orientation
        self.positionSelf = Position(pos.pose.pose.position.x, pos.pose.pose.position.y)
        
    def positionAndOrientationTargetCallback(self, target_pos):
        #calc orientation from pose information
        fake_odom=target_pos.pose.pose
        euler=euler_from_quaternion([fake_odom.orientation.x,fake_odom.orientation.y,fake_odom.orientation.z,fake_odom.orientation.w])
        orientation =euler[2]

        self.orientationTarget = orientation
        self.positionTarget = Position(target_pos.pose.pose.position.x, target_pos.pose.pose.position.y)
        

    def sonar_callback(self, current_sonar_scan):
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points

        #berechnung des Abstands
        self.sonar_ranges = np.zeros(len(self.sonar_angles))
        for i in range(0, len(self.sonar_angles)):
            self.sonar_ranges[i] = np.sqrt(sonar_points[i].x**2 + sonar_points[i].y**2)
    


if __name__ == '__main__':

    rospy.init_node("Behaviour")

    try:
        node = Behaviour(RoboterType.CAT)
    except rospy.ROSInterruptException:
        print("ROS Interruption")
