#! /usr/bin/env python
import numpy as np
import rospy
import math
import exercise_1
import analog_gates

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class SimpleHoming:

    def __init__(self):
        # the final position
        final_position = np.array([-2,-3])

        #initialize position
        self.position  = np.random.normal(0,1,2)
        self.orientation = np.random.normal(0,1,1)
        '''
        the subscriber should be implemented here.
        it should read in the odometry data and write the position to 
        self.position and the orientation to self.orientation.
        to achieve this the callback function needs to be a function of the class
        Simple Homing.
        Tutorial for Python Classes:
        https://www.w3schools.com/python/python_classes.asp
        '''

        rospy.Subscriber("dead_reckoning", Pose, self.callback)

        pub            = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():   
            #output=Twist()

            output = closed_loop(self.position,self.orientation,final_position)
            combination = combine(output, CollisionAvoidance())
            pub.publish(combination)

    def callback(self, pos):
        self.orientation[0] = pos.orientation.z
        self.position[0] = pos.position.x
        self.position[1] = pos.position.y


def closed_loop(position,orientation,target):
        output=Twist()
        '''
        TODO
        Function needs to be implemented in such a way that a Twist Message 
        is returned which can control the robot in such a way that he 
        arrives at his target and remains there.
        For a reference on how to write a Twist message one can consult 
        exercise_1.py
        '''

        target_rel_x = target[0] - position[0]
        target_rel_y = target[1] - position[1]

        dir_x = np.cos(orientation[0]-np.pi/2)*target_rel_x + np.sin(orientation[0]-np.pi/2)*target_rel_y
        dir_y = np.cos(orientation[0]-np.pi/2)*target_rel_y - np.sin(orientation[0]-np.pi/2)*target_rel_x


        #rospy.loginfo([dir_x, dir_y])

        #diff_x = target[0] - position[0]
        #diff_y = target[1] - position[1]
        if np.abs(np.arctan2(dir_x, dir_y)) < np.pi/4 and dir_x*dir_x+dir_y*dir_y > 0.01:
            output.linear.x = 0.5
        else:
            output.linear.x = 0
        if np.abs(np.arctan2(dir_x,dir_y)) > np.pi/30:
            output.angular.z = np.arctan2(dir_x, dir_y)/3

        return output

def combine(homing, coll):
    output = Twist()
    output.linear.x = invoke_gate(homing.linear.x, 1-coll[0])
    output.angular.z = invoke_gate(homing.angular.z, coll[1])
    return output






if  __name__=="__main__":
    rospy.init_node("homing")
    try:
        node=SimpleHoming()

    except rospy.ROSInterruptException:
        pass
