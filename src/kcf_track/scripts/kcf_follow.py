#!/usr/bin/env python

import rospy
import thread, threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
angle=[0.0]*3
distan=[0.0]*3
class Follower:
	def __init__(self):
	
	        self.dis_Kp       = rospy.get_param('~dis_Kp')
		self.dis_Kd       = rospy.get_param('~dis_Kd')
                self.dis_setPoint = rospy.get_param('~dis_setPoint')
                self.dis_last_error = 0

	        self.ang_Kp       = rospy.get_param('~ang_Kp')
		self.ang_Kd       = rospy.get_param('~ang_Kd')
                self.ang_setPoint = rospy.get_param('~ang_setPoint')
                self.ang_last_error = 0

		self.line_max_speed     = rospy.get_param('~line_maxSpeed') 
		self.angular_max_speed  = rospy.get_param('~angular_maxSpeed') 
		#self.controllButtonIndex = rospy.get_param('~controllButtonIndex')

		#self.buttonCallbackBusy=False
		#self.active=False
		#self.i=0

		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size =3)
		
		self.positionSubscriber = rospy.Subscriber('/kcf/track', Twist, self.positionUpdateCallback)
              
	def trackerInfoCallback(self, info):
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		rospy.logwarn(info.data)

	def positionUpdateCallback(self, twist):

	   angleX= twist.angular.z
	   distance = twist.linear.x

           dis_error = self.dis_setPoint - distance
	   ang_error = self.ang_setPoint - angleX
           
           linearSpeed  = -dis_error*self.dis_Kp - self.dis_last_error*self.dis_Kd
           angularSpeed =  ang_error*self.ang_Kp + self.ang_last_error*self.ang_Kd

           if(linearSpeed>self.line_max_speed):
	      linearSpeed=self.line_max_speed
           if(linearSpeed<-self.line_max_speed):
	      linearSpeed=-self.line_max_speed
           if(angularSpeed>self.angular_max_speed):
	      angularSpeed=self.angular_max_speed
           if(angularSpeed<-self.angular_max_speed):
	      angularSpeed=-self.angular_max_speed
           if(angleX==0):
              angularSpeed=0.0
           if(distance==0):
              linearSpeed=0.0

           self.dis_last_error = dis_error
           self.ang_last_error = ang_error
	   # create the Twist message to send to the cmd_vel topic
	   velocity = Twist()	
	   velocity.linear = Vector3(linearSpeed,0,0.)
	   velocity.angular= Vector3(0., 0.,angularSpeed)
	   rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))

	   self.cmdVelPublisher.publish(velocity)
			

if __name__ == '__main__':
	
	print('starting')
	rospy.init_node('follower')
	follower = Follower()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')

