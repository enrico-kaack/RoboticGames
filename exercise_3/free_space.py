#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



'''
Die gesammte Kollisionsvermeidung ist in einer Klasse verpackt um die momentanigen Geschwindigkeitsdaten aus dem callback des
Geschwindigkeitssubscribers herauszuholen und diese in der berechnung der neuen Richtgeschwindigkeit zu benutzen.
alternativ haetten hier auch globale Variabeln verwendet werden koennen, diese Methode wird in der Community allerdings als
eleganter angesehen.
'''
class FreeSpace:

    def __init__(self):

        self.current_vel_x = 0.0
        self.current_ang_z = 0.0


        '''
        Die verwendung eines Kraftbasierten Ansatzes bedeutet, dass die momentane Geschwindigkeit
        modifiziert wird.
        Dazu muss sie allerdings zunaechst bekannt sein.
        Der Roboter in der Simulation stellt diese bereits ueber einen sogenannten Subscriber
        zur verfuegung. Ein Tutorium ist auf folgender Website zu finden:
        http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
        '''
        rospy.Subscriber("/p3dx/p3dx_velocity_controller/odom", Odometry, self.velocity_callback)
        rospy.Subscriber("/robotic_games/sonar", PointCloud, self.sonar_callback)

        '''
        Das Ergebniss der Berechnung wird dem Roboter als soll-geschwindigkeit zurueckgegeben.
        dies passiert ueber einen sogenannten Publisher
        (siehe wieder http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29 )
        geregelt, der Name des Topics wird dabei von der Simulation vorgegeben.
        '''
        self.col_avoid_publisher = rospy.Publisher("/p3dx/p3dx_velocity_controller/cmd_vel", Twist, queue_size=10)

        rospy.spin()

    def velocity_callback(self, current_odometry):
        self.current_vel_x = current_odometry.twist.twist.linear.x
        self.current_ang_z = current_odometry.twist.twist.angular.z

    def sonar_callback(self, current_sonar_scan):
        adjustment = Twist()
        
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points
        # Die Orientierung der einzelnen Sensoren folgt:
        sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        sonar_angles = sonar_angles / 360.0 * 2 * np.pi

        #berechnung des Abstands
        sonar_ranges = np.zeros(len(sonar_angles))
        for i in range(0, len(sonar_angles)):
            sonar_ranges[i] = np.sqrt(sonar_points[i].x**2 + sonar_points[i].y**2)

        biggest_distance = np.amax(sonar_ranges)
        # check from which angle this small range comes from
        indices_biggest_ranges = np.argmax(sonar_ranges)
        id_biggest_range = indices_biggest_ranges if not isinstance(indices_biggest_ranges, list) else indices_biggest_ranges[0]
        rospy.loginfo(id_biggest_range)
        
        if id_biggest_range > 3:
            adjustment.angular.z = -2
        else:
            adjustment.angular.z = 2
        
        adjustment.linear.x  = 0.5
        self.col_avoid_publisher.publish(adjustment)


if __name__ == '__main__':

    rospy.init_node("FreeSpace")

    try:
        node = FreeSpace()
    except rospy.ROSInterruptException:
        pass
