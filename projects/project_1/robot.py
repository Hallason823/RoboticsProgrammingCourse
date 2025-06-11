import numpy as np
import matplotlib.pyplot as plt
from environment import env

"""Creating the class bot, in positional arguments (e.g., positions), the first one (0) represents the horizontal/row position, although the
second one ((1) represents the vertical/column position. In its inicialitizion, it required offer the sensor_field, the attribute sensor_field offer 
should be greater or equal than 1 to make sense. Also,it creted an attribiute of erro (starts as None)."""

class bot:
	def __init__(self, sensor_field):
		self.robot_env = None
		self.initial_position = []
		self.final_position = []
		self.current_position = []
		self.planned_historic = []
		self.displacement = []
		self.sensor_field = sensor_field
		self.numb_of_dect_obstacles = None
		self.values_table = None
		self.planned_path = []
		self.erro = None

		'''In this method, it's possible to chosse a position to the bot, if k == 0, then initial and first current position receive the 
position, in case contrary, then final position receives it.'''
	def choose_position (self, position, real_env, k):
#Checking if position has the proper size.
		if len(position) == 2:
#Checking if exist an obstacle in the informed position and, also, sorting out in (depending of k): (1) the initial and first current 
#position with its historic (when k == 0) and (2) the final position (when k != 0), finally, it updated the atrribute of erro.
			if real_env.map[position[0]][position[1]] != -1 and k == 0:
				self.initial_position = position
				self.current_position = position
				self.displacement.append(position)
				self.erro = 0
			elif real_env.map[position[0]][position[1]] != -1 and k != 0:
				self.final_position = position
				self.erro = 0
			else:
#Message of erro, it shows the entered position is occupied, as well as, it updated the attribute of erro.
				print('The entered position is occupied !\n Give a validy entry, please.\n')
				self.erro = 1
		else:
#Message of erro, it shows the entered isn't a 2D position, as well as, it updated the attribute of erro.
			print('Erro! Invalidy entry!\n')
			self.erro = 1

#In this method, it's possible to detect the robot's field view.
	def detecting_obstacles (self, real_env):
#Checking if the current position is known
		if self.current_position != []:
			self.robot_env = env(real_env.row, real_env.column)
#It updated the attribute of erro.
			self.erro = 0
#It's going through all possible detectable positions from the robot.
			for i in range(-self.sensor_field, self.sensor_field+1):
				for j in range(-self.sensor_field, self.sensor_field+1):
#Checking if the position belong the environment of the robot.
					if 0 <= self.current_position[0] + i < self.robot_env.row and 0 <= self.current_position[1] + j < self.robot_env.column:
#If the position belong the map, then now, it checks if this position isn't free in the map of the environment.
						if real_env.map[self.current_position[0]+i][self.current_position[1]+j] == -1:
#If it's occupied, it inserted this information in the map of the robot.
							self.robot_env.map[self.current_position[0]+i][self.current_position[1]+j] = -1
#Counting the number of obstacles
			self.numb_of_dect_obstacles  = np.count_nonzero(np.array(self.robot_env.map))
		else:
#Message of erro, if the current position isn't known, also, it updated the attribute of erro.
			print('The current position is not defined !\nAdd it, please.')			
			self.erro = 1

	def update_values_table(self):
		self.values_table = None
#Checking if the final position is known.
		if self.final_position != []:
#It updated the attribute of erro.
			self.erro = 0
#Copying the map of the robot's environment to values table (or potential table).
			self.values_table = np.zeros((self.robot_env.row, self.robot_env.column))
			for i in range(self.robot_env.row):
				for j in range(self.robot_env.column):
					self.values_table[i][j] = self.robot_env.map[i][j]
#It's indicating the final position
			self.values_table [self.final_position[0]][self.final_position[1]] = 1
#Interactor variable, also it starts with 1 which it corresponds the value attribute to the final position.
			interactor = 1
#Stopped variable which it is limited by the total number map's elements
			stop = self.numb_of_dect_obstacles+1
			while stop < self.robot_env.row * self.robot_env.column:
#Finding the interactor in the map.
				for i in range(self.robot_env.row):
					for j in range(self.robot_env.column):
						if self.values_table[i][j] == interactor:
#Selecting the close elements (top, bottom and sides).
							for p,q in zip([-1,1,0,0],[0,0,-1,1]):
#Checking if each element belongs the map.
								if 0 <= i+p < self.robot_env.row and 0 <= j +q < self.robot_env.column:
#Checking if each element is free.
									if self.values_table[i+p][j+q] == 0:
#If it gets to arrive here, it sums interactor+1 in this position, also, it sums +1 in the stopping criterion.
										self.values_table[i+p][j+q] = interactor+1
										stop += 1
#Now, the interactor sums +1.
				interactor += 1
		else:
#Message of erro, it shows the final position is not defined, also, it updated the attribute of erro.
			print('The final position is not defined!\nAdd it, please.')
			self.erro = 1


	def planning(self):
#Checking if the current position is known.
		if self.current_position != []:
#Inicialitizing the planned path as an empty list, and then, it updates the planned path with the kown informations.
			self.planned_path = []
#The planned path inicialitizes with the current_position.
			self.planned_path.append([self.current_position[0], self.current_position[1]])
#It investigates the number of the neighbors closer.
			route_value = self.values_table[self.current_position[0]][self.current_position[1]]-1
#It should stop when it to be "in front of" onf neighbors with the smallest number (i.e., 1).
			while route_value > 1:
#Looking for the neighbors of the last inserted position in the planned path.
				i,j = self.planned_path[-1]
				indexes = list(zip([-1,1,0,0],[0,0,-1,1]))
#It chooses the nearest neighbor at random.
				np.random.shuffle(indexes)
				for p,q in indexes:
#Checking if exists the neighbor in the robot map.
					if 0 <= i+p < self.robot_env.row and 0 <= j +q < self.robot_env.column:
#Checking if the neighbor is a true neigbhor.
						if self.values_table[i+p][j+q] == route_value:
#If yes, it adds the position.
							self.planned_path.append([i+p, j+q])
#Also, it finds the neighbor of this true neighbor.
							route_value -= 1
#The last position in planned path is obviously the final position.
			self.planned_path.append([self.final_position[0], self.final_position[1]])

		else:
#Message of erro, it shows the current position is not defined, also, it updated the attribute of erro.
			print('The current position is not defined!\nAdd it, please.')
			self.erro = 1

#In this method, it detects collision, if exists a possible collision in the position = [position[0], position[1]], then it returns True.
#Basically, it checks if the position in the map is occupied which it symbolizer by the integer -1.

	def collision_detection (self, position):
		return self.robot_env.map[position[0]][position[1]] == -1

#In this method, it updated the current position and its historic.

	def change_position (self, position):
		self.current_position = position
		self.displacement.append(position)

#In this method, we are doing the total planning, basically, we use other methods of the class bot.
	def total_planning(self, real_env):
#This contains each map of the bot and planning,
		self.planned_historic = []
#Detecting obstacles.
		self.detecting_obstacles(real_env)
#Stopping when the robot arrives at the finish.
		while self.current_position != self.final_position:
#Uppading the values table.
			self.update_values_table()
#Making a planning
			self.planning()
#Variable of detection of obstacles.
			detection = False
#Variable that it scrolls through the list and indicates if we've come all the path.
			k = 1
#Adding the robot map and planned path in the historic.
			self.planned_historic.append([self.robot_env, self.planned_path])
#The loop stop when the robot detected any obstacle or arrived at the finish.
			while k < len(self.planned_path) and detection == False:
#Checking if it occurred a collision.
				if self.collision_detection(self.planned_path[k]) == True:
					detection = True
				else:
#If not, it continues the path.
					self.change_position(self.planned_path[k])
#It scrolls to the next element of the list.
				k += 1
#Updating the view of robot.
				self.detecting_obstacles(real_env)
				
#Using the method env.plotEnv.
	def  plotRobEnv(self, time, real_env):
		self.robot_env.plotEnv('bot')

#Ploting the planning map	
	def  plotPlanningMap(self, path):
		matrix = np.zeros((self.robot_env.row, self.robot_env.column))
		color = 1
#Adjusting figure size and change the title.
		plt.figure(figsize=(16,7.5))
		plt.get_current_fig_manager().set_window_title('The planning map')
#Adding labels to x and y axis.
		plt.xlabel("ROWS", size = 14)
		plt.ylabel("COLUMNS", size= 14)
#Adjusting the ticks on both x and y axis.
		plt.xticks([i for i in range(self.robot_env.row)], size=11, color = "red")
		plt.yticks([i for i in range(self.robot_env.column)], size=11, color = "red")
#Each position in the planning is given a different color.
		for position in path:
			matrix[position[0]][position[1]] = color
			color += 1
		plt.imshow(matrix)
#Displaying it.
		plt.show()