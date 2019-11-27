import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import rosbag

spawn_point=np.array([[0,0],[0,1.36],[0,1.23],[-2.1,2.1],[0,2.18]])
bag_mouse = rosbag.Bag(os.path.expanduser('~/.ros/mouse_path.bag'))
bag_cat = rosbag.Bag(os.path.expanduser('~/.ros/cat_path.bag'))
bag_cat = rosbag.Bag(os.path.expanduser('~/.ros/path.bag'))
point_list_mouse=np.array([[0,0]])
point_list_cat=np.array([[0,0]])
mouse_time=[]
cat_time=[]
all_topics=['/mouse/base_pose_ground_truth','/cat/base_pose_ground_truth']
for topic, msgs, t in bag_mouse.read_messages(topics=['/mouse/base_pose_ground_truth']):
	point_list_mouse=np.append(point_list_mouse,[[msgs.pose.pose.position.x,msgs.pose.pose.position.y]],axis=0)
	mouse_time.append(t)
for topic, msgs, t in bag_cat.read_messages(topics=['/cat/base_pose_ground_truth']):
	point_list_cat=np.append(point_list_cat,[[msgs.pose.pose.position.x,msgs.pose.pose.position.y]],axis=0)
	cat_time.append(t)
point_list_cat=list(point_list_cat)
cat_time=list(cat_time)


point_list_cat.pop(0)
while len(point_list_cat) > len(point_list_mouse):
	point_list_cat.pop(0)
	cat_time.pop(0)

point_list_cat=np.array(point_list_cat)
cat_time=np.array(cat_time)

plt.scatter(point_list_mouse[1:,1],point_list_mouse[1:,0],label="mouse")
plt.scatter(point_list_cat[1:,1],point_list_cat[1:,0],label="cat")
plt.ylabel("Y Position")
plt.xlabel("X Position")
plt.title("Pfad der Roboter")
plt.axis('equal')
plt.legend()
plt.savefig("report_pfad.png", dpi=600)
plt.clf()


diff = (point_list_cat- point_list_mouse)**2


plt.plot( np.sqrt(diff[:,0]+diff[:,1]),label="Roboterabstand")
plt.title("Roboterabstand")
plt.ylabel("Distanz")
plt.xlabel("Zeit")
plt.legend()
plt.savefig("report_distance.png", dpi=600)
plt.clf()





