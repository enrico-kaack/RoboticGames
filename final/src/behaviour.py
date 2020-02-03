#!/usr/bin/env python

import numpy as np
import rospy
import math
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
    MOUSE = 0
    CAT = 1

class Behaviour:

    def __init__(self, roboterType):
        
        #constants equal for both robots
        self.sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        self.sonar_angles = self.sonar_angles / 360.0 * 2 * np.pi
        self.steps = 6
        self.choices = [Velocity(1, -2), Velocity(1, 0.0), Velocity(1, 2.0)]
        
        #constants different for both robots
        self.maxLinVel = [0.35,0.4]
        self.maxAngVel = [1.0,0.8]
        
        
        #properties for both robots
        self.positions = [Position(0.0,0.0),Position(0.0,0.0)]
        self.currentVelocities = [Velocity(0.0, 0.0),Velocity(0.0, 0.0)]
        self.orientations = [0.0,0.0]
        self.last_angle = 0.0
        self.sonar_ranges = np.array([np.zeros(len(self.sonar_angles)),np.zeros(len(self.sonar_angles))])

        
        #properties for this robot
        self.roboterType = roboterType
        self.pub = None

        #ros callback listener
        rospy.Subscriber("/cat/p3dx_velocity_controller/odom", Odometry, self.catVelocityCallback)
        rospy.Subscriber("/cat/base_pose_ground_truth", Odometry, self.catPositionAndOrientationCallback)
        rospy.Subscriber("/cat/sonar", PointCloud, self.catSonarCallback)
        rospy.Subscriber("/mouse/p3dx_velocity_controller/odom", Odometry, self.mouseVelocityCallback)
        rospy.Subscriber("/mouse/base_pose_ground_truth", Odometry, self.mousePositionAndOrientationCallback)
        rospy.Subscriber("/mouse/sonar", PointCloud, self.mouseSonarCallback)
        
        if self.roboterType == RoboterType.CAT:
            #ros publisher to give control commands
            self.pub = rospy.Publisher("/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

        else:
            #ros publisher to give control commands
            self.pub = rospy.Publisher("/mouse/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

        #init main loop
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():                          
            bestCombination = self.calcBestCombination(self.positions, self.orientations, self.steps)
            toGo = self.choices[bestCombination[0][self.roboterType]]
            collAvoidanceOutput = self.performCollisionAvoidances(toGo)
            output = Twist()
            output.linear.x = np.interp(collAvoidanceOutput.linear, [-1,1], [-self.maxLinVel[self.roboterType], self.maxLinVel[self.roboterType]])
            output.angular.z = np.interp(collAvoidanceOutput.angular, [-1,1], [-self.maxAngVel[self.roboterType], self.maxAngVel[self.roboterType]])
            print(toGo)
            self.pub.publish(output)
            
    """
    positions = [mousePosition,catPosition], orientations = [mouseOrientation, catOrientation], stepCount = recursive depth of calculation
    """
    def calcBestCombination(self, positions, orientations, stepCount):
        if (stepCount == 1):
            simulatedPositions, simulatedOrientations = self.simulateStep(positions,orientations)
            costs = []
            for y in range(len(simulatedPositions[0])):
                costLine = []
                for x in range(len(simulatedPositions[0])):
                    nextCost = self.costFunction([simulatedPositions[0][y],simulatedPositions[1][x]], [simulatedOrientations[0][y],simulatedOrientations[1][x]])
                    costLine.append(nextCost)
                costs.append(costLine)
            return self.nash(costs)
        else: 
            simulatedPositions, simulatedOrientations = self.simulateStep(positions,orientations)
            costs = []
            for y in range(len(simulatedPositions[0])):
                costLine = []
                for x in range(len(simulatedPositions[0])):
                    irrelevantDecision, nextCost = self.calcBestCombination([simulatedPositions[0][y],simulatedPositions[1][x]], [simulatedOrientations[0][y],simulatedOrientations[1][x]], stepCount-1)
                    costLine.append(nextCost)
                costs.append(costLine)
            return self.nash(costs)
        
    """
    returns a single cost for one pair of positions and orientations of both players; the higher the cost the worse for the cat
    """
    def costFunction(self, positions, orientations):
        constMaxAngVelToDegreesInPi = math.pi/2
        constMaxLinVelToCoordinatesPerSecond = 0.2
        
        catPos = positions[RoboterType.CAT]
        mousePos = positions[RoboterType.Mouse]
        catOrientation = orientations[RoboterType.CAT]
        targetOrientation = math.arctan2(mousePos[0]-catPos[0],mousePos[1]-catPos[1])
        turnTime = (targetOrientation-catOrientation)/(self.maxAngVel[RoboterType.CAT]/constMaxAngVelToDegreesInPi)
        distance = math.sqrt(pow(mousePos[0]-catPos[0],2),pow(mousePos[1]-catPos[1],2))
        distanceTime = distance/(self.maxLinVel[RoboterType.CAT]*constMaxLinVelToCoordinatesPerSecond)
        cost = turnTime+distanceTime
        return cost

    def simulateStep(self, positions,orientations):
        simulatedPositions = [None, None]
        simulatedOrientations = [None,None]
        for i in range(2):
            simulatedPositions[i], simulatedOrientations[i] = self.simulatePosition(i, positions[i], orientations[i])
        return simulatedPositions, simulatedOrientations

    def simulatePosition(self, roboterType, position, orientation):
        #discretizing all possibiliies into 3 different directions: left, straight, right
        
        #calcualte the new positions
        #deltaX = arcsin w (w = angle) * d (distance in m, assume the distance is smaller when turning)
        #deltaY = arccos w (w = angle) * d
        w = np.pi * 0.5 + np.array([self.choices[0].angular, self.choices[1].angular, self.choices[2].angular]) * self.maxAngVel[self.roboterType] + np.repeat(-1*orientation, 3) #world orientation with own variation
        h = np.array([self.choices[0].linear, self.choices[1].linear, self.choices[2].linear]) * self.maxLinVel[self.roboterType]


        deltaX = np.sin(w) * h
        deltaY = np.cos(w) * h

        currentX = np.repeat(position.x, 3) #turn first scalar into array with n arguments
        currentY = np.repeat(position.y, 3)

        newPosX = currentX + deltaX
        newPosY = currentY + deltaY
        newPos = []
        for (index, posX) in enumerate(newPosX):
            newPos.append(Position(posX, newPosY[index]))

        #estimate new orientation
        newOrientation = w #could be improved
        return np.array([newPos, newOrientation])
        
    def velocityCallback(self, current_odometry, roboterType):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z
        
    def positionAndOrientationCallback(self, pos, roboterType):
        #calc orientation from pose information
        fake_odom=pos.pose.pose
        euler=euler_from_quaternion([fake_odom.orientation.x,fake_odom.orientation.y,fake_odom.orientation.z,fake_odom.orientation.w])
        orientation =euler[2]

        self.orientations[roboterType] = orientation
        self.positions[roboterType] = Position(pos.pose.pose.position.x, pos.pose.pose.position.y)
        
    def mousePositionAndOrientationCallback(self,pos):
        self.positionAndOrientationCallback(pos,RoboterType.MOUSE)
        
    def catPositionAndOrientationCallback(self,pos):
        self.positionAndOrientationCallback(pos,RoboterType.CAT)

    def sonarCallback(self, currentSonarScan, roboterType):
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonarPoints = currentSonarScan.points

        #berechnung des Abstands
        self.sonar_ranges[roboterType] = np.zeros(len(self.sonar_angles))
        for i in range(0, len(self.sonar_ranges[roboterType])):
            self.sonar_ranges[roboterType][i] = np.sqrt(sonarPoints[i].x**2 + sonarPoints[i].y**2)
    
    def mouseSonarCallback(self, currentSonarScan):
        self.sonarCallback(currentSonarScan,RoboterType.MOUSE)
        
    def catSonarCallback(self, currentSonarScan):
        self.sonarCallback(currentSonarScan,RoboterType.CAT)

    def catVelocityCallback(self, current_odometry):
        self.currentVelocities[1] = Velocity(current_odometry.twist.twist.linear.x, current_odometry.twist.twist.angular.z)

    def mouseVelocityCallback(self, current_odometry):
        self.currentVelocities[0] = Velocity(current_odometry.twist.twist.linear.x, current_odometry.twist.twist.angular.z)
    """
    costs for player 0 are wins for player 1; returns the ids the palyer take as an array and the cost for player 0 for that decision
    """
    def nash(self,costs):
        #mouseNashID:
        mins = []
        for y in range(len(costs)):
            minimum = costs[y][0]
            for x in range(len(costs)):
                if costs[y][x] < minimum:
                    minimum = costs[y][x]
            mins.append(minimum)
        mouseNashID = 0
        mouseMaxMin = mins[0]
        for i in range(len(mins)):
            if mins[i] > mouseMaxMin:
                mouseMaxMin = mins[i]
                mouseNashID = i
        #catNashID:
        maxes = []
        for x in range(len(costs)):
            maximum = costs[0][x]
            for y in range(len(costs)):
                if costs[y][x] > maximum:
                    maximum = costs[y][x]
            maxes.append(maximum)
        catNashID = 0
        catMinMax = maxes[0]
        for i in range(len(maxes)):
            if maxes[i] < catMinMax:
                catMinMax = maxes[i]
                catNashID = i
        return [mouseNashID,catNashID],costs[catNashID][mouseNashID]
    
    def performCollisionAvoidances(self, toGo):
        collAvoidance = self.calculate_collision_avoidance()
        distanceToTarget = self.positions[self.roboterType].distanceTo(self.positions[1-self.roboterType])
        output = Velocity(0,0)
        output.linear = toGo.linear

        if np.abs(np.min(self.sonar_ranges) - distanceToTarget)<0.1:  #smallest sonar distance inside the range with distance to target
            output.angular = toGo.angular
        else:
            output.angular = prevail_gate(collAvoidance.angular, toGo.angular)
        return output
    
    def getWallDistance(self):
        distances = [np.min([self.sonar_ranges[0], self.sonar_ranges[1]]), self.sonar_ranges[2],np.min([self.sonar_ranges[3], self.sonar_ranges[4]]), self.sonar_ranges[5], np.min([self.sonar_ranges[6], self.sonar_ranges[7]])]
        distances = -0.7*np.sqrt(np.array(distances)) + 200 #parameter for collision avoidance
        return distances
    
    def calculate_collision_avoidance(self):
        for i, sonar_range in enumerate(self.sonar_ranges[self.roboterType]):
            if sonar_range == 0.0:
                rospy.logerr('Catched Zero')
                self.sonar_ranges[self.roboterType][i] = 1e-12

        threshold = 0.4
        sonar_ranges = self.sonar_ranges[self.roboterType]
        if np.min([sonar_ranges[0], sonar_ranges[1], sonar_ranges[2], sonar_ranges[3], sonar_ranges[4], sonar_ranges[5], sonar_ranges[6], sonar_ranges[7]]) < threshold:
            
            #threshold on force = distance
            distancesWithThreshold = (1/sonar_ranges * -1) - threshold

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
        velocity_adjustment.angular = rotationAngle*2

        return velocity_adjustment
import sys
if __name__ == '__main__':

    rospy.init_node("Behaviour")
    

    try:
        if len(sys.argv) > 1 and sys.argv[1] == "cat":
            node = Behaviour(RoboterType.CAT)
        else:
            node = Behaviour(RoboterType.MOUSE)

    except rospy.ROSInterruptException:
        print("ROS Interruption")