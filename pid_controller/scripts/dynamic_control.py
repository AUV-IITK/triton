#!/usr/bin/python
import numpy as np
import rospy

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from underwater_sensor_msgs.msg import DVL  
from PID import PIDRegulator

class controlPID:	
	def __init__(self,x,y,z):
		self.cur_pos=np.zeros(3)
		self.cur_vel=np.zeros(3)
		self.ref=np.zeros(3)
		self.ref[0]=x
		self.ref[1]=y
		self.ref[2]=z
		
	def update(self):
		
		ref=np.zeros(3)
		u=np.zeros(5)
		
		kp=1
		ki=1
		kd=0.1
		t=0.01
		
		pid=PIDRegulator(kp,ki,kd)
		
		sub_pose=rospy.Subscriber('g500/pose',Pose,self.pose_callback)
		sub_vel=rospy.Subscriber('g500/dvl',DVL,self.vel_callback)
		pub_acc=rospy.Publisher('g500/thrusters_input', Float64MultiArray, queue_size=10)
		 
		e_pos=self.ref-self.cur_pos
		cmd_vel=pid.regulate(e_pos,t)
		e_vel=cmd_vel-self.cur_vel
		cmd_acc=pid.regulate(e_vel,t)
		
		u[0]=-cmd_acc[1]
		u[1]=-cmd_acc[1]
		u[2]=-cmd_acc[2]
		u[3]=-cmd_acc[2]
		u[4]=-cmd_acc[0]
		
		msg=Float64MultiArray()
		msg.data=u
		pub_acc.publish(msg)
		print self.cur_pos

	def vel_callback(self, msg):
		self.cur_vel = np.array([msg.bi_x_axis, msg.bi_y_axis, msg.bi_z_axis])

	def pose_callback(self,msg):
		p=msg.position
		self.cur_pos=np.array([p.x, p.y, p.z])


print "Controller Node:"
rospy.init_node('Controller', anonymous=True)
x=int(input("Enter x co-ordinate"))
y=int(input("Enter y co-ordinate"))
z=int(input("Enter z co-ordinate"))
cnt=controlPID(x,y,z)
while not rospy.is_shutdown():
	cnt.update()
	rospy.Rate(100).sleep()

