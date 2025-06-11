#!/usr/bin/env python3
import rospy 
import time
import numpy as np
from sensor_msgs.msg import LaserScan

class subpub:

	def __init__(self):
		rospy.Subscriber('scaninformation', LaserScan, self.callback, queue_size=1)
		
	def callback(self,msg):
		self.count = msg
		print(self.count)	
		
run = subpub()
rospy.init_node('subsssss')		
rospy.spin()