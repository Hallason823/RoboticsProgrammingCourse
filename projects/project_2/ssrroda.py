#!/usr/bin/env python3
import rospy 
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32MultiArray

class count:
	def __init__(self,p1,p2,p3,p4):
		self.pub = rospy.Publisher('publish', Float32MultiArray, queue_size=1)
		self.counter = Float32MultiArray()
		self.counter.data = [0]*2
		self.count_left = 0
		self.count_right = 0
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(p1, GPIO.IN)
		GPIO.setup(p2, GPIO.IN)
		GPIO.setup(p3, GPIO.IN)
		GPIO.setup(p4, GPIO.IN)	
		GPIO.add_event_detect(p1,GPIO.RISING,callback=self.count_1)	
		GPIO.add_event_detect(p2,GPIO.RISING,callback=self.count_1)
		GPIO.add_event_detect(p3,GPIO.RISING,callback=self.count_2)
		GPIO.add_event_detect(p4,GPIO.RISING,callback=self.count_2)
		
	def count_1(self,channel):
		self.count_right += 1
		
	def count_2(self,channel):
		self.count_left += 1
	
	def pub_counter(self):
		self.counter.data[0] = self.count_left
		self.counter.data[1] = self.count_right	
		self.pub.publish(self.counter)
		
p1,p2,p3,p4= 5,6,17,27
rospy.init_node('Codificador')
my_count = count(p1,p2,p3,p4)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
	my_count.pub_counter()
	rate.sleep()
	print('Left {}' .format(my_count.count_left))
	print('Right {}' .format(my_count.count_right))