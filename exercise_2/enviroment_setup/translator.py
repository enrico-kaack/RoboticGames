#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32

def translate_pose(data):
    pub=rospy.Publisher("/p3dx/p3dx_velocity_controller/odom",Odometry,queue_size=1)
    pub.publish(data)
    pub_two=rospy.Publisher("/p3dx/p3dx/base_ground_truth",Odometry,queue_size=1)
    pub_two.publish(data)



def translate_sonar(data):
    pub=rospy.Publisher("/robotic_games/sonar",PointCloud,queue_size=1)
    pub.publish(data)


def translate_cmd_vel(data):
    pub=rospy.Publisher("/RosAria/cmd_vel",Twist,queue_size=1)
    pub.publish(data)



def translator():
    rospy.init_node("translator")
    rospy.Subscriber("/RosAria/pose",Odometry,translate_pose)
    rospy.Subscriber("/RosAria/sonar",PointCloud,translate_sonar)
    rospy.Subscriber("/p3dx/p3dx_velocity_controller/cmd_vel",Twist,translate_cmd_vel)
    rospy.spin()

if __name__=="__main__":
    try:
        translator()
    except rospy.ROSInterruptException:
        pass

