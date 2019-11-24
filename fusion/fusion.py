#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import analog_gates
from tf.transformations import euler_from_quaternion


class Fusion:

    def __init__(self):
        self.current_vel_x = 0.0
        self.current_ang_z = 0.0

        self.last_angle = 0.0

        self.position  = np.random.normal(0,1,2)
        self.orientation = np.random.normal(0,1,1)

        self.sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        self.sonar_angles = self.sonar_angles / 360.0 * 2 * np.pi
        self.sonar_ranges = np.zeros(len(self.sonar_angles))

        self.final_position = np.array([-2,-3])


        rospy.Subscriber("/cat/p3dx_velocity_controller/odom", Odometry, self.velocity_callback)
        rospy.Subscriber("/cat/sonar", PointCloud, self.sonar_callback)

        rospy.Subscriber("/cat/base_pose_ground_truth", Odometry, self.position_callback)
        pub = rospy.Publisher("/cat/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/mouse/base_pose_ground_truth", Odometry, self.target_position_callback)
        

        while not rospy.is_shutdown():   
            output=Twist()
            collAvoidance = self.calculate_collision_avoidance()
            homing = self.calculate_homing()
            freeSpace = self.calculate_freespace()

            output = self.fusion(collAvoidance, homing, freeSpace)
            #rospy.loginfo(output)

            pub.publish(output)

    def target_position_callback(self, target_pos):
        self.final_position[0] = target_pos.pose.pose.position.x
        self.final_position[1] = target_pos.pose.pose.position.y

    def velocity_callback(self, current_odometry):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points

        #berechnung des Abstands
        self.sonar_ranges = np.zeros(len(self.sonar_angles))
        for i in range(0, len(self.sonar_angles)):
            self.sonar_ranges[i] = np.sqrt(sonar_points[i].x**2 + sonar_points[i].y**2)

    def position_callback(self, pos):
        fake_odom=pos.pose.pose
        euler=euler_from_quaternion([fake_odom.orientation.x,fake_odom.orientation.y,fake_odom.orientation.z,fake_odom.orientation.w])
        orientation =euler[2]

        self.orientation[0] = orientation
        self.position[0] = pos.pose.pose.position.x
        self.position[1] = pos.pose.pose.position.y

    def fusion(self, collAvoidance, homing, freeSpace):
        output = Twist()

        output.linear.x = analog_gates.and_gate(homing.linear.x, freeSpace.linear.x)
        output.angular.z = analog_gates.and_gate(homing.angular.z, freeSpace.angular.z)
        temp = output
        #rospy.loginfo(collAvoidance)
        output.linear.x = analog_gates.prevail_gate(1-collAvoidance.linear.x, output.linear.x)
        output.angular.z = analog_gates.prevail_gate(collAvoidance.angular.z, output.angular.z)
        print("\t|".join(["\t".join([str(output.linear.x), str(output.angular.z)]), "\t".join([str(temp.linear.x), str(temp.angular.z)]), "\t".join([str(collAvoidance.linear.x), str(collAvoidance.angular.z)]), "\t".join([str(homing.linear.x), str(homing.angular.z)]), "\t".join([str(freeSpace.linear.x), str(freeSpace.angular.z)])]))

        #rospy.loginfo("DONE FUSION")
        return output
    
    def calculate_collision_avoidance(self):
        """
        sum = np.zeros(2)

        for i, self.sonar_range in enumerate(self.sonar_ranges):
            if self.sonar_range == 0.0:
                rospy.logerr('Catched Zero')
                self.sonar_range = 1e-12
            vec = np.array([1/self.sonar_range * np.cos(self.sonar_angles[i]), 1/self.sonar_range * np.sin(self.sonar_angles[i])])
            sum += vec

        sum *= (-1)

        sum_dist = np.linalg.norm(sum)

        if sum_dist < 10.0:
            t =  Twist()
            t.linear.x = 1
            return t

        phi = -np.arctan2(sum[1], sum[0])

        force = np.zeros(2)
        force[0] = np.clip(-sum[0] / 20, 0, 1)
        force[1] = np.clip(phi, -np.pi/2, np.pi/2)

        if -force[1] == self.last_angle:
            force[1] = self.last_angle

        self.last_angle = force[1]

        velocity_adjustment = Twist()
        velocity_adjustment.linear.x  = 1-force[0]
        velocity_adjustment.angular.z = force[1]

        """

        for i, self.sonar_range in enumerate(self.sonar_ranges):
            if self.sonar_range == 0.0:
                rospy.logerr('Catched Zero')
                self.sonar_ranges[i] = 1e-12

        threshold = 0.4
        if np.min([self.sonar_ranges[2], self.sonar_ranges[3], self.sonar_ranges[4], self.sonar_ranges[5]]) < 0.4:
            
            #threshold on force = distance
            distancesWithThrehold = (1/self.sonar_ranges * -1) - threshold

            weight = np.array([0.2, 0.3, 0.4, 0.8, 0.8, 0.4, 0.3, 0.2])

            b = np.sin(self.sonar_angles) * distancesWithThrehold * weight
            a = np.cos(self.sonar_angles) * distancesWithThrehold * weight

            F_x = np.sum(b)
            F_y = np.sum(a)

            rotationAngle = np.arctan(F_y / F_x)


            #linearForce = 1/((self.sonar_ranges[3] + self.sonar_ranges[4]) / 2) 
            
            #if linearForce < 0.1:
            #    linearForce = 0.1
            linearForce = 1
        else:
            linearForce = 0
            rotationAngle = 0

        velocity_adjustment = Twist()
        velocity_adjustment.linear.x  = 1-linearForce
        velocity_adjustment.angular.z = rotationAngle

        return velocity_adjustment

    def calculate_freespace(self):
        adjustment = Twist()
        biggest_distance = np.amax(self.sonar_ranges)

        # check from which angle this small range comes from
        indices_biggest_ranges = np.argmax(self.sonar_ranges)
        id_biggest_range = indices_biggest_ranges if not isinstance(indices_biggest_ranges, list) else indices_biggest_ranges[0]
        
        if id_biggest_range > 3:
            adjustment.angular.z = -1
        else:
            adjustment.angular.z = 1
        
        adjustment.linear.x  = 0.5
        return adjustment


    def calculate_homing(self):
        output=Twist()

        target_rel_x = self.final_position[0] - self.position[0]
        target_rel_y = self.final_position[1] - self.position[1]

        dir_x = np.cos(self.orientation[0]-np.pi/2)*target_rel_x + np.sin(self.orientation[0]-np.pi/2)*target_rel_y
        dir_y = np.cos(self.orientation[0]-np.pi/2)*target_rel_y - np.sin(self.orientation[0]-np.pi/2)*target_rel_x


        #rospy.loginfo([dir_x, dir_y])

        #diff_x = target[0] - position[0]
        #diff_y = target[1] - position[1]
        if np.abs(np.arctan2(dir_x, dir_y)) < np.pi/4 and dir_x*dir_x+dir_y*dir_y > 0.01:
            output.linear.x = 0.5
        else:
            output.linear.x = 0.2
        if np.abs(np.arctan2(dir_x,dir_y)) > np.pi/30:
            output.angular.z = np.arctan2(dir_x, dir_y)

        return output

if __name__ == '__main__':

    rospy.init_node("Fusion")

    try:
        node = Fusion()
    except rospy.ROSInterruptException:
        pass
