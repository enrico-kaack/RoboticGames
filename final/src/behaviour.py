#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
#import analog_gates
from tf.transformations import euler_from_quaternion

class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return "{}|{}".format(self.x , self.y)

    def distanceTo(target):
        return np.sqrt(np.sqr(target.x - self.x) + np.sqr(target.y- self.y) )

class Velocity:
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular

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

        self.roboterType = roboterType
        self.pub = None

        if self.roboterType == RoboterType.CAT:
        #ros callback listener for own information
            rospy.Subscriber("/cat/p3dx_velocity_controller/odom", Odometry, self.velocitySelfCallback)
            rospy.Subscriber("/cat/base_pose_ground_truth", Odometry, self.positionAndOrientationSelfCallback)
            rospy.Subscriber("/cat/sonar", PointCloud, self.sonar_callback)
            
            #ros callback listener for target information
            rospy.Subscriber("/mouse/base_pose_ground_truth", Odometry, self.positionTargetCallback)

            #ros publisher to give control commands
            self.pub = rospy.Publisher("/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        else:
            #ros callback listener for own information
            rospy.Subscriber("/mouse/p3dx_velocity_controller/odom", Odometry, self.velocitySelfCallback)
            rospy.Subscriber("/mouse/base_pose_ground_truth", Odometry, self.positionAndOrientationSelfCallback)
            rospy.Subscriber("/mouse/sonar", PointCloud, self.sonar_callback)
            
            #ros callback listener for target information
            rospy.Subscriber("/cat/base_pose_ground_truth", Odometry, self.positionAndOrientationTargetCallback)

            #ros publisher to give control commands
            self.pub = rospy.Publisher("/mouse/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)


        #init main loop
        while not rospy.is_shutdown():   
            self.simulatePositions()

    """"
    Returns the simulated positions in order: self, target
    """"
    def simulatePositions(self):
        if self.roboterType == RoboterType.CAT:
            cat = simulatPosition(RoboterType.CAT, self.positionSelf, self.orientationSelf)
            mouse = simulatePosition(RoboterType.MOUSE, self.positionTarget, self.orientationTarget)
            return cat, mouse
        else: 
            cat = simulatPosition(RoboterType.CAT, self.positionTarget, self.orientationTarget)
            mouse = simulatePosition(RoboterType.MOUSE, self.positionSelf, self.orientationSelf)
            return mouse, cat

    def simulatePosition(self, roboterType, position, orientation):
        #discretizing all possibiliies into 5 different directions: straight, half-left, left (respective for right) 
        
        #calcualte the new positions
        #deltaX = arcsin w (w = angle) * d (distance in m, assume the distance is smaller when turning)
        #deltaY = arccos w (w = angle) * d
        if roboterType == RoboterType.CAT:
            w = np.array([-1.0, -0.6, 0.0, 0.6, 1.0]) + np.repeat(orientation, 5) #world orientation with own variation
            h = np.array([0.7, 0.9, 1.0, 0.9, 0.7])
        else:
            w = np.array([-1.0, -0.6, 0.0, 0.6, 1.0]) + np.repeat(orientation, 5) #world orientation with own variation
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
        return np.arra([newPos, newOrientation])
        



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
        fake_odom=pos.pose.pose
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
