#!/usr/bin/env python3
import rospy 
import time
import numpy as np
import RPi.GPIO as GPIO
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
#from blabla import Twist

class base_controller:

	def __init__(self, sinais_por_rotacao, frequence, dist_rodas, raio_rodas, razaopowervelocity,kp,ki,kd,pwleft,pwright,pwplusleft,pwplusright):
		#rospy.Subscriber('desejo', Twist, self.callback_velocity, queue_size=1)
		rospy.Subscriber('publish', Float32MultiArray, self.callback_encoder, queue_size=1)
		rospy.Subscriber('scaninformation', LaserScan, self.callback_laserlidar, queue_size=1)
		#self.pub = rospy.Publisher('pubvelocidade', Twist, queue_size=1)
		self.sinais_por_rotacao = sinais_por_rotacao
		self.frequence = frequence
		self.VelAng = [0]*2
		self.initial = True
		self.velocidade_desejada = [0]*2
		self.dist_rodas = dist_rodas
		self.raio_rodas = raio_rodas
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.erro_ant = [0]*2
		self.erro = [0]*2
		self.control = [0]*2
		self.w = [0]*2
		self.razaopowervelocity = razaopowervelocity
		self.pwright = pwright
		self.pwleft = pwleft
		self.pwplusright = pwplusright
		self.pwplusleft = pwplusleft
		self.dutycycle = [80]*2
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pwleft, GPIO.OUT)
		GPIO.setup(self.pwright, GPIO.OUT)
		GPIO.setup(self.pwplusleft, GPIO.OUT)
		GPIO.setup(self.pwplusright, GPIO.OUT)
		self.pwm16 = GPIO.PWM(self.pwleft,1000)
		self.pwm16.start(self.dutycycle[0])
		self.pwm22 = GPIO.PWM(self.pwright,1000)
		self.pwm22.start(self.dutycycle[1])
		self.velocidade_desejada[0] = 4
		self.velocidade_desejada[1] = 4
		
	def teste_run_off(self):
		GPIO.output(self.pwplusleft,GPIO.LOW)
		GPIO.output(self.pwplusright,GPIO.LOW)
		
	def teste_run_on(self):
		GPIO.output(self.pwplusleft,GPIO.HIGH)
		GPIO.output(self.pwplusright,GPIO.HIGH)
	
	def controladorpid(self):
		self.erro[0] = self.velocidade_desejada[0] - self.VelAng[0]
		self.erro[1] = self.velocidade_desejada[1] - self.VelAng[1]
		self.control[0] = self.kp * self.erro[0] #+ self.ki*((self.erro_ant[0] + self.erro[0])/2) * (1/self.frequence) + self.kd*(self.erro[0] - self.erro_ant[0])*self.frequence
		self.control[1] = self.kp * self.erro[1] #+ self.ki*((self.erro_ant[1] + self.erro[1])/2) * (1/self.frequence) + self.kd*(self.erro[1] - self.erro_ant[1])*self.frequence
		self.w[0] = (self.velocidade_desejada[0]+self.control[0])*self.razaopowervelocity
		self.w[1] = (self.velocidade_desejada[1]+self.control[1])*self.razaopowervelocity
		if self.w[0] > 100 or self.w[1] > 100:
			if self.w[0]>self.w[1]:
				self.w[1] = 100 *(self.w[1]/self.w[0])
				self.w[0] = 100
			else: 
				self.w[0] = 100 *(self.w[0]/self.w[1])
				self.w[1] = 100
		self.pwm16.ChangeDutyCycle(self.w[0])		
		self.pwm22.ChangeDutyCycle(self.w[1])
		self.erro_ant = self.erro
		
	def callback_velocity(self,msg):	
		#self.velocidadelinear = msg.linear.x
		#self.velocidadeangular = msg.angular.z
		self.velocidade_desejada[0] = (self.velocidadelinear - (self.dist_rodas * self.velocidadeangular))/(self.raio_rodas)  
		self.velocidade_desejada[1] = (self.velocidadelinear + ((self.dist_rodas/2) * self.velocidadeangular))/(self.raio_rodas) 
		 			
	def callback_encoder(self,msg):
		self.SinalCount = msg.data
		if self.initial == True: 
			self.SinalAnt = msg.data
			self.initial = False
			self.erro_ant = [0]*2
		self.VelAng[0] = ((self.SinalCount[0] - self.SinalAnt[0]) * 2 * np.pi) / ((self.sinais_por_rotacao)*(1/self.frequence))  
		self.VelAng[1] = ((self.SinalCount[1] - self.SinalAnt[1]) * 2 * np.pi) / ((self.sinais_por_rotacao)*(1/self.frequence))
		self.SinalAnt = self.SinalCount
		print(self.VelAng)
	
	def callback_laserlidar(self,msg):
		self.laser = msg
		#self.laserranges = msg.ranges
		#self.laser_angulo_max = msg.angle_max
		#self.laser_angulo_min = msg.angle_min
		#self.laser_angulo_increment = msg.angle_increment
		for i in range(int((self.laser.angle_max - self.laser.angle_min)/(self.laser.angle_increment)+1)):
				if self.laser.ranges[i] == 0:
					self.laser.ranges[i] = np.inf
		for i in range(int((self.laser.angle_max - self.laser.angle_min)/(self.laser.angle_increment)+1)):
			if self.laser.ranges[i] <= 1000:
				self.teste_run_off()
			#else:	
			#	self.teste_run_on()	
		
pwleft,pwright,pwplusleft,pwplusright = 16,22,26,24
run = base_controller(1012, 10, 0.2185, 0.0335, 100/6.2,1.2, 0, 0, pwleft, pwright, pwplusleft, pwplusright)
rospy.init_node('Base_controller')		
run.teste_run_on()
timeout = time.time() + 15
while time.time() < timeout:
	x=1
	run.controladorpid()	 		
run.teste_run_off()