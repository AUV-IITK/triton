#!/usr/bin/python

from numpy import *
from numpy.linalg import inv
from numpy.linalg import det
import numpy as np


class Kalman_filter:
	def __init__(self,X,U,cur_vel,cur_pos):
		self.X=X
		self.row_main=6
		self.col_main=1
		self.cur_vel=cur_vel
		self.cur_pos=cur_pos
		self.dt=0.01
		self.loop_iter=50

		###initialization of state matrices
		self.P=np.eye(self.X.shape[0])
		#set covariance in diagonal elements of P(by default setting it to 0)
		for i in range(6):
			self.P[i][i]=0

		self.A=np.eye(self.X.shape[0])
		for i in range(3):
			self.A[i][i+3]=self.dt

		#set covariance in diagonal elements of Q(by defualt setting is 0)
		self.Q=np.eye(self.X.shape[0])
		for i in range(6):
			self.Q[i][i]=0

		self.B=np.ones((6,1))
		for i in range(3):
			self.B[i][0]=0.5*self.dt*self.dt
			self.B[i+3][0]=self.dt

		self.U=U

		###Measurement matrices
		self.Y=np.zeros((6,1))
		for i in range(3):
			self.Y[i][0]=self.cur_pos[i]
			self.Y[i+3][0]=self.cur_vel[i]

		self.H=self.A

		self.R=np.eye(self.X.shape[0])

	
	def print_data(self):
		print("checking of correct sensor data")
		print(self.Y)
	

	#velocity subscriber callback
	def vel_callback(self,msg):
		self.cur_vel =np.array([msg.bi_x_axis, msg.bi_y_axis, msg.bi_z_axis])
					
			
	#postion subscriber callback	
	def pose_callback(self,msg):
		self.cur_pos=np.array([msg.position.x, msg.position.y, msg.position.z])

	#Algorithm of Kalman_filter
	def kf_predict(self):
		for i in range(6):
			self.B[i]=self.B[i]*self.U
		self.X=np.dot(self.A,self.X)+self.B
		self.P=np.dot(self.A,np.dot(self.P,self.A.T)) + self.Q
		return(self.X,self.P)		

	def kf_update(self):
		IM=np.dot(self.H,self.X)
		IS=self.R+np.dot(self.H,np.dot(self.P,self.H.T))
		K=np.dot(self.P,np.dot(self.H.T,inv(IS)))
		self.X=self.X+np.dot(K,(self.Y-IM))
		self.P=self.P-np.dot(K,np.dot(IS,K.T))
		LH=self.gauss_main(self.Y,IM,IS)
		return (self.X,self.P,K,IM,IS,LH)

	def gauss_main(self,X,M,S):
		if M.shape[1]==1:
			DX=X-np.tile(M,X.shape[1])
			E=0.5*np.sum(DX*(np.dot(inv(S),DX)),axis=0)
			E=E+0.5*M.shape[0]*log(2-pi) + 0.5*log(det(S))
			P=np.exp(-E)

		elif X.shape[1]==1:
			DX=np.tile(X,M.shape[1])-M
			E=0.5*np.sum(DX*(np.dot(inv(S),DX)),axis=0)
			E=E+0.5*M.shape[0]*log(2-pi) + 0.5*log(det(S))
			P=np.exp(-E)

		else:
			DX=X-M
			E=0.5*np.dot(DX.T,dot(inv(S),DX))
			E=E+0.5*M.shape[0]*log(2-pi) + 0.5*log(det(S))
			P=np.exp(-E)

		return(P[0],E[0])

	

	def kf_main(self):
		for i in range(self.loop_iter):
			(self.X,self.P)=self.kf_predict()
			(self.X,self.P,K,IM,IS,LH)=self.kf_update()

		return (self.X)

