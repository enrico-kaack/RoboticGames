import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import itertools

if len(sys.argv) != 2:
	print("specify the base folder as first parameter. Exiting")
	sys.exit(1)
base_path = sys.argv[1]

spawn_point=np.array([[0,0],[0,1.36],[0,1.23],[-2.1,2.1],[0,2.18]])
bag_mouse = rosbag.Bag(os.path.expanduser(base_path + '/mouse/mouse_path.bag'))
bag_cat = rosbag.Bag(os.path.expanduser(base_path + '/cat/cat_path.bag'))

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
point_list_mouse = list(point_list_mouse)
mouse_time = list(mouse_time)

point_list_cat.pop(0)
while len(point_list_cat) > len(point_list_mouse):
	point_list_cat.pop(0)
	cat_time.pop(0)

while len(point_list_mouse) > len(point_list_cat):
	point_list_mouse.pop(0)
	mouse_time.pop(0)

point_list_cat=np.array(point_list_cat)
cat_time=np.array(cat_time)
point_list_mouse = np.array(point_list_mouse)
mouse_time = np.array(mouse_time)


def adjust_lightness(color, amount=0.5):
	#<1 makes color brighter, >1 makes color darker
    import matplotlib.colors as mc
    import colorsys
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], max(0, min(1, amount * c[1])), c[2])

colorModifier = np.linspace(0.3,1.7,len(point_list_mouse))
colorsMouse = map(lambda mod: adjust_lightness('blue', mod), colorModifier)
colorsCat = map(lambda mod: adjust_lightness('orange', mod), colorModifier)




plt.scatter(point_list_mouse[1:,1],point_list_mouse[1:,0],label="mouse", color=colorsMouse)
plt.scatter(point_list_cat[1:,1],point_list_cat[1:,0],label="cat", color=colorsCat)
plt.ylabel("Y Position")
plt.xlabel("X Position")
plt.title("Pfad der Roboter")
plt.axis('equal')
plt.legend()

ax = plt.gca()
leg = ax.get_legend()
leg.legendHandles[0].set_color('blue')
leg.legendHandles[1].set_color('orange')

plt.savefig("report_pfad" + base_path + ".png", dpi=1000)
plt.clf()


diff = (point_list_cat- point_list_mouse)**2


plt.plot( np.sqrt(diff[:,0]+diff[:,1]),label="Roboterabstand")
plt.title("Roboterabstand")
plt.ylabel("Distanz")
plt.xlabel("Zeit")
plt.legend()
plt.savefig("report_distance"+base_path + ".png", dpi=1000)
plt.clf()





