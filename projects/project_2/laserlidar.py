#!/usr/bin/env python3
import os
import ydlidar
import time
import sys
import rospy
from matplotlib.patches import Arc
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from sensor_msgs.msg import LaserScan

rospy.init_node('Lidar')
pub = rospy.Publisher('scaninformation', LaserScan, queue_size=1)
infolidar = LaserScan()
RMAX = 32.0
fig = plt.figure()
fig.canvas.set_window_title('YDLidar LIDAR Monitor')
lidar_polar = plt.subplot(polar=True)
lidar_polar.autoscale_view(True,True,True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)
ports = ydlidar.lidarPortList();
port = "/dev/ydlidar";
infolidar.angle_min = (-120*np.pi)/180
infolidar.angle_max = (120*np.pi)/180
infolidar.angle_increment = (0.5*np.pi)/180
infolidar.time_increment = 1/10
infolidar.time_increment = 3
infolidar.range_min = 0
infolidar.range_max = 1
ydlidar.os_init () ;
ports = ydlidar.lidarPortList () ;
port = " / dev / ydlidar " ;
for key , value in ports.items () :
	port = value ;
	print ( port ) ;
laser = ydlidar.CYdLidar () ;
laser.setlidaropt ( ydlidar.LidarPropSerialPort , port ) ;
laser.setlidaropt ( ydlidar.LidarPropSerialBaudrate , 115200) ;
laser.setlidaropt ( ydlidar.LidarPropLidarType , ydlidar.TYPE_TRIANGLE ) ;
laser.setlidaropt ( ydlidar.LidarPropDeviceType , ydlidar.YDLIDAR_TYPE_SERIAL ) ;
laser.setlidaropt ( ydlidar.LidarPropScanFrequency , 10.0) ;
laser.setlidaropt ( ydlidar.LidarPropSampleRate , 3) ;
laser.setlidaropt ( ydlidar.LidarPropSingleChannel , True ) ;
laser.setlidaropt ( ydlidar.LidarPropMaxAngle , 180.0) ;
laser.setlidaropt ( ydlidar.LidarPropMinAngle , -180.) ;
laser.setlidaropt ( ydlidar.LidarPropMaxRange , 1.0) ;
laser.setlidaropt ( ydlidar.LidarPropMinRange , 0.08) ;
laser.setlidaropt ( ydlidar.LidarPropIntenstiy , False ) ;
ret = laser.initialize () ;
if ret :
	ret = laser.turnOn () ;
	scan = ydlidar.LaserScan () ;
	if ret:
		timeout = time.time() + 30
		dumb = True
		# while dumb:
			# print("dumb")
		while time.time() < timeout:
			r = laser.doProcessSimple(scan);
			print(len(scan.points))
			if r:
				angle = []
				ran = []
				intensity = []
				for point in scan.points:		
					print(point.angle*180/np.pi, point.range)
					if (-1 * point.angle*180/np.pi + 120) > 180:
						teta = -1 * point.angle*180/np.pi - 240
					else:
						teta = -1 * point.angle*180/np.pi + 120
					if -120 <= (teta) <= 120:
						if (point.range <= 100 and point.range > 0.0):
							angle.append(teta); #correcao 
							print((teta))
							ran.append(point.range);
							intensity.append(point.intensity);
						else:
							angle.append(teta); #correcao
							print((teta))
							ran.append(np.inf);
							intensity.append(0);
			# dumb = False				
			anginit = 120 ; 
			incremento = 0.5; 
			angmax = 120; 
			angmin = -120; 
			numerodeiteracoes = ((angmax-angmin)/incremento) + 1; 
			vector_size = len(angle);
			for i in range(int(numerodeiteracoes)):
				found = False
				j = 0
				while j < vector_size and found == False:
					if(abs(anginit-angle[j])<0.5):
						infolidar.ranges.append(ran[j])
						infolidar.intensities.append(intensity[j])
						found == True
					if (j == vector_size -1 and found == False):
						infolidar.ranges.append(np.inf)					
						infolidar.intensities.append(0)
					j += 1
				anginit -= incremento 
			for i in range(int(numerodeiteracoes)):
				if infolidar.ranges[i] == 0:
					infolidar.ranges[i] = np.inf
			#infolidar.ranges = ran
			#infolidar.intensities = intensity	
			pub.publish(infolidar)
			infolidar.ranges = []
			infolidar.intensities = []
#while ret and ydlidar. os_isOk () :
#	r = laser.doProcessSimple ( scan ) ;
#	if r :
#		for point in scan.points:
#			print("{}   {}".format(point.angle, point.range));
#	else :
#		print ( " Failed to get Lidar Data " )
#	time.sleep (0.05) ;
	laser.turnOff () ;
laser.disconnecting () ;