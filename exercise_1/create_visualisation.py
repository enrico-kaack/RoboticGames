import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import rosbag

spawn_point=np.array([[0,0],[0,1.36],[0,1.23],[-2.1,2.1],[0,2.18]])
for i in range(1,6):
    bag = rosbag.Bag(os.path.expanduser('~/.ros/robot_path'+str(i)+'.bag'))
    point_list=np.array([[0,0]])
    all_topics=['/p3dx/p3dx/base_pose_ground_truth']
    for specific_topics in all_topics:
        for topic, msgs, t in bag.read_messages(topics=[specific_topics]):
            point_list=np.append(point_list,[[msgs.pose.pose.position.x,msgs.pose.pose.position.y]],axis=0)

    background=plt.imread("worlds/2D-Model/rgarena"+str(i)+".png")
    plt.imshow(background,extent=[-3.14+spawn_point[i-1,0],3.14+spawn_point[i-1,0],-3.14+spawn_point[i-1,1],3.14+spawn_point[i-1,1]])
    plt.scatter(-point_list[1:,1],point_list[1:,0])
    #plt.show()
    plt.savefig("/home/enrico/catkin_ws/src/exercise_1/"+str(i)+".jpg")
    plt.clf()




