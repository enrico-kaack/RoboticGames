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
class CollisionAvoidance:

    def __init__(self):

        self.current_vel_x = 0.0
        self.current_ang_z = 0.0

        self.last_angle = 0.0


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
        # Die Sonarsensoren des Roboters werden im folgenden Array gespeichert
        sonar_points = current_sonar_scan.points
        # Die Orientierung der einzelnen Sensoren folgt:
        sonar_angles = np.array([-90.0, -50.0, -30.0, -10.0, 10.0, 30.0, 50.0, 90.0])
        sonar_angles = sonar_angles / 360.0 * 2 * np.pi

        #berechnung des Abstands
        sonar_ranges = np.zeros(len(sonar_angles))
        for i in range(0, len(sonar_angles)):
            sonar_ranges[i] = np.sqrt(sonar_points[i].x**2 + sonar_points[i].y**2)

        #Kraft welche auf Roboter wirkt
        force=self.calculate_force(sonar_angles,sonar_ranges)
        '''
        ROS kommuniziert ueber feste vordefinierte Datenpakete, Messages genannt.
        fuer Geschwindigkeiten wird dafuer haeufig der sogenannte Twist verwendet.

        Naehere informationen zum Twist sind unter:
        https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        zu finden.

        Da es sich hier um ein 2-Dimensionales System handelt, wird lediglich die linear-
        Geschwindigkeit in x und die Winkelgeschwindigkeit in z Richtung verwendet.
        '''
        velocity_adjustment = Twist()
        velocity_adjustment.linear.x  = 1-force[0]
        velocity_adjustment.angular.z = force[1]
        self.col_avoid_publisher.publish(velocity_adjustment)

    def calculate_force(self,sonar_angles,sonar_ranges):
        ''' 
        HIER KOMMT DER CODE HIN

        Das Ergebniss der Berechnungen soll in Form eines 2-dimensionalen
        Arrays zurueckgegeben werden.
        Die 1. Komponente entspricht dabei der Aenderung der Lineargeschwindigkeit
        und die 2. enstprechend der Aenderung der Winkelgeschwindigkeit.
        '''

        sum = np.zeros(2)

        for i, sonar_range in enumerate(sonar_ranges):
            if sonar_range == 0.0:
                rospy.logerr('Catched Zero')
                sonar_range = 1e-12
            vec = np.array([1/sonar_range * np.cos(sonar_angles[i]), 1/sonar_range * np.sin(sonar_angles[i])])
            sum += vec

        sum *= (-1)

        sum_dist = np.linalg.norm(sum)

        if sum_dist < 10.0:
            return np.zeros(2)

        phi = -np.arctan2(sum[1], sum[0])

        force = np.zeros(2)
        force[0] = np.clip(-sum[0] / 20, 0, 1)
        force[1] = np.clip(phi, -np.pi/2, np.pi/2)

        if -force[1] == self.last_angle:
            force[1] = self.last_angle

        self.last_angle = force[1]

        return force



if __name__ == '__main__':

    rospy.init_node("CollisionAvoidance")

    try:
        node = CollisionAvoidance()
    except rospy.ROSInterruptException:
        pass
