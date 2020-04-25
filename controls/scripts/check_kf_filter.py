#! /usr/bin/python
print("node started")

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from underwater_sensor_msgs.msg import DVL 


import kalman_filter

rospy.init_node('check', anonymous=True)

print("imported kalman_filter")

X=np.zeros((6,1))
U=np.ones(1)

cur_vel=np.array([0.0,0.0,0.0])
cur_pos=np.array([0.0,0.0,0.0])
def vel_callback(msg):
		cur_vel =np.array([msg.bi_x_axis, msg.bi_y_axis, msg.bi_z_axis])
					
			
def pose_callback(msg):
		cur_pos=np.array([msg.position.x, msg.position.y, msg.position.z])



def main():
	sub_pose=rospy.Subscriber('g500/pose',Pose,pose_callback,queue_size=1)
	sub_vel=rospy.Subscriber('g500/dvl',DVL,vel_callback,queue_size=1)
	kf=kalman_filter.Kalman_filter(X,U,cur_vel,cur_pos)
	print("sensor data")
	print(cur_pos)
	kf.print_data()
	print("result")
	print(kf.kf_main())

while not rospy.is_shutdown():
	print("entering main")
	main()
	rospy.Rate(100).sleep()
