#!/usr/bin/python
import numpy as np
import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from underwater_sensor_msgs.msg import DVL  
from PID import PIDRegulator

class controlPID:	
	def __init__(self):
		self.cur_pos=np.zeros(3)
		self.cur_vel=np.zeros(3)
		
	def update(self):
		rospy.init_node('Controller', anonymous=True)
		
		ref=np.zeros(3)
		
		
		kp=1
		ki=0
		kd=1
		t=0.01
		
		ref[0]=int(input("Enter x co-ordinate"))
		ref[1]=int(input("Enter y co-ordinate"))
		ref[2]=int(input("Enter z co-ordinate"))
		
		pid=PIDRegulator(kp,ki,kd)
		
		sub_pose=rospy.Subscriber('g500/pose',Pose,self.pose_callback)
		sub_vel=rospy.Subscriber('g500/dvl',DVL,self.vel_callback)
		pub_acc=rospy.Publisher('g500/thruster_input', Float64MultiArray, queue_size=10)
		 
		e_pos=ref-self.cur_pos
		cmd_vel=pid.regulate(e_pos,t)
		e_vel=cmd_vel-self.cur_vel
		cmd_acc=pid.regulate(e_vel,t)
		pub_acc.publish(cmd_acc)

	#def PID:


	def vel_callback(self, msg):
		self.cur_vel = numpy.array([msg.bi_x_axis, msg.bi_y_axis, msg.bi_z_axis])

	def pose_callback(self,msg):
		p=msg.Point
		self.cur_pos=numpy.array([p.x, p.y, p.z])

cnt=controlPID()
print "Controller Node:"
cnt.update()
