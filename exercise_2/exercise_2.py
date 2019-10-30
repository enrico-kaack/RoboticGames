#! /usr/bin/env python
import numpy as np
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist


class SimpleHoming:

    def __init__(self):
        # the final position
        final_position = np.array([0,4])

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

        rospy.Subscriber("dead_reckoning", Pose, self.dead_reckoningCallback)

        pub = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():   
            output=Twist()
            output = closed_loop(self.position,self.orientation,final_position)
            pub.publish(output)

    def dead_reckoningCallback(self, data):
        self.position = np.array([data.position.x, data.position.y])
        self.orientation = data.orientation.z
        rospy.loginfo(self.position)


def closed_loop(position,orientation,target):
        output=Twist()
        '''
        TODO
        Function needs to be implemented in such a way that a Twist Message 
        is returned which can controll the robot in such a way that he 
        arrives at his target and remains there.
        For a reference on how to write a Twist message one can consult 
        exercise_1.py
        '''

        if np.linalg.norm(position - target) < 0.1:
            return output 
        
        ####calc rotation
        #make target relativ to robot in cartesian
        targetRelative = target - position
        #convert to polar
        rotation = np.arctan2(targetRelative[1], targetRelative[0])

        ####calc speed
        #calc distance
        distance = np.sqrt(targetRelative[0] ** 2 + targetRelative[1] ** 2)

        output.linear.x = 0.2
        output.angular.z = rotation - (np.pi - orientation) 
        print(output)
        return output

if  __name__=="__main__":
    rospy.init_node("homing")
    try:
        node=SimpleHoming()

    except rospy.ROSInterruptException:
        pass
