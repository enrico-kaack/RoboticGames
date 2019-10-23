import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import rosbag

spawn_point=np.array([[0,0],[0,1.36],[0,1.23],[-2.1,2.1],[0,2.18]])
bag = rosbag.Bag(os.path.expanduser('~/.ros/homing_path.bag'))
point_list=np.array([[0,0]])
all_topics=['/p3dx/p3dx/base_pose_ground_truth']
for specific_topics in all_topics:
    for topic, msgs, t in bag.read_messages(topics=[specific_topics]):
        point_list=np.append(point_list,[[msgs.pose.pose.position.x,msgs.pose.pose.position.y]],axis=0)

plt.scatter(point_list[1:,1],point_list[1:,0])
plt.ylabel("Y Position")
plt.xlabel("X Position")
plt.title("Pfad des Roboters")
plt.axis('equal')
plt.show()

time_steps=np.arange(0,len(point_list[1:,1]))
plt.scatter(time_steps,point_list[1:,0],label="X Achse")
plt.scatter(time_steps,point_list[1:,1],label="Y Achse")
plt.ylabel("Position")
plt.xlabel("Zeit")
plt.title("X und Y Sprungantwort")
plt.legend()
plt.show()






