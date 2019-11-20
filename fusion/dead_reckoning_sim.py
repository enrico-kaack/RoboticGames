#! /usr/bin/env python
import sys
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

'''
node calculates fake odometry results using base_pose_ground_truth.
Normally /p3dx/p3dx_velocity_controller/odom would be more appropriate since it simulates wheel sensors.
this however can lead to problems while testing the code since manual movement of the roboter in gazeboo would
not the odometrically determined position.
The Node also reads the variance of a standart gaussian and adds it as noise to each measurment.
This is once again a simplification as the error accumulates while driving and would potentionally get bigger and bigger
'''
def noise_addition():
    rospy.Subscriber("/p3dx/p3dx/base_pose_ground_truth",Odometry,calc_distance,float(sys.argv[1]))
    rospy.spin()

def calc_distance(data,var):
    pub=rospy.Publisher("dead_reckoning",Pose,queue_size=1)
    fake_odom=data.pose.pose
    fake_odom.position.x=fake_odom.position.x+np.random.normal(0,np.sqrt(var),1)
    fake_odom.position.y=fake_odom.position.y+np.random.normal(0,np.sqrt(var),1)
    '''
    The standart formulation of a robots orientation is in terms of the so called Quaternions,
    which are a 4 Dimensional extension to the complex numbers.
    for more Info: 
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#targetText=Compared%20to%20Euler%20angles%20they,numerically%20stable%2C%20and%20more%20efficient.&targetText=When%20used%20to%20represent%20rotation,represent%20the%203D%20rotation%20group.
    It is not necessary to understand in detail how these work because we convert them to the more intuitive euler angles.
    We dont run into any of the troubles described in Section 5 of the Wikipedia Article since we are describing a System with only two Spatial Dimensions.
    '''
    euler=euler_from_quaternion([fake_odom.orientation.x,fake_odom.orientation.y,fake_odom.orientation.z,fake_odom.orientation.w])
    fake_odom.orientation.z=euler[2]+np.random.normal(0,np.sqrt(var),1)
    pub.publish(fake_odom)

if __name__=="__main__":
    rospy.init_node("dead_reckoning")
    try:
        noise_addition()
    except rospy.ROSInterruptException:
        pass
