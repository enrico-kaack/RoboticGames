#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from angles import two_pi_complement

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

angles = np.linspace(-90, 90, num=8)
cmd_vel_publisher = rospy.Publisher('/mouse/p3dx_velocity_controller/cmd_vel', Twist, queue_size=10)

def callback(data):
	distances = []
	for point in data.points:
		distances.append(np.sqrt(point.x**2+point.y**2))
	min_distance = min(distances)
 
	cmd_vel = Twist()
	if min_distance <= 0.3:
		if distances.index(min_distance) <= 5:
			cmd_vel.linear.x = 0
			cmd_vel.angular.z = -0.8
		else:
			cmd_vel.linear.x = 0
			cmd_vel.angular.z = 0.8
		cmd_vel_publisher.publish(cmd_vel)
	else:
		cmd_vel.linear.x = 1
		cmd_vel_publisher.publish(cmd_vel)



if __name__ == '__main__':
    try:
        rospy.init_node('random_walk', anonymous=True)
        #rospy.Subscriber("/mouse/p3dx_velocity_controller/cmd_vel", Twist, update_curr_cmd_vel)
        rospy.Subscriber("/mouse/sonar", PointCloud, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
