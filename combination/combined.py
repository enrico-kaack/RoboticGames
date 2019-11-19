#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from exercise_1 import ColisionAvoidance
from exercise_2 import SimpleHoming
from exercise_3 import FreeSpace




class Combination:

    def __init__(self):

        self.current_vel_x = 0.0
        self.current_ang_z = 0.0

        #initialize position
        self.position  = np.random.normal(0,1,2)
        self.orientation = np.random.normal(0,1,1)

        final_position = np.array([-2,-3])

        self.colisionAvoidance = ColisionAvoidance()
        self.simpleHoming = SimpleHoming()
        self.freeSpace = FreeSpace()


        #rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom", Odometry, self.velocity_callback)
        #rospy.Subscriber("/robotic_games/sonar", PointCloud, self.sonar_callback)

        rospy.Subscriber("dead_reckoning", Pose, self.callback)


        self.col_avoid_publisher = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        pub = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)


        rospy.spin()



if __name__ == '__main__':

    rospy.init_node("Combination")

    try:
        node = Combination()
    except rospy.ROSInterruptException:
        pass
