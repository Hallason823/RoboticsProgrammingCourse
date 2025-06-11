import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors


"""Creation of env class. In its inicialitizion, It required to offer the number of rows and columns, these informations are two important
attributes of the class, i.e., they represent the dimensions of the map. The last attributes are the map, the mobile_obbstacles and the erro (which it starts as
None), basically, the map starts with an empty map, then it adds the null rows (one by one) with the attributte append from the class list,
finally, [0]*column represents the creation of null row. Also, the mobile_obstacles contains the location of the mobile obstacles, it starts as
 an empty list"""

class env:
	def __init__(self, row, column):
		self.row = row
		self.column = column
		self.map = []
		self.mobile_obstacles = []
		for i in range(row):
			self.map.append([0]*column)
		self.erro = None

	"""In this method, it adds or removes an obstacle of the map, it depends of the k. Indeed, when k == '0', then it adds an obstacle which
it represents by -1,  else (i.e., if k is any other thing), then it removes an obstacle which it represents by 0. In this method, it
received two other informations, specifically, two lists (row and column) that should to have lenght between 1 and 2 (included) each
one, otherwise, it shows the following message: Erro! Invalidy entry!, also, it change the value of erro to 1. If the lenght of row
and column are suitables, then it modifies the map the following way: (1) the dimension of row and column are unitary, then it modifies
the element of map in the position (row,column); (2) the dimension of row is unitary and the other isn't, then it modifies the row from
the range between column[0] and column[1] -1 (included) - vice versa; (3) the dimensions of row and column aren't unitary, then it 
modifies the retangle [row[0] x column [0]; row[1]-1 x column[1]-1]. In the case of dimension 2, it's interesting  note that it should 
change the entries to a range with the  command range(entry[0], entry [1]), to difference if the dimension is 1 or 2, it executes a 
ternany by lenght(entry), returning the conversion in an auxiliary variable that will be traversed by the 'for'."""

	def k_obstacle(self, row, column, k):
		if 0 < len(row) <= 2 and 0 < len(column) <= 2 :
			rows = row if len(row) == 1 else range(row[0], row[1])
			columns = column if len(column) == 1 else range(column[0], column[1])
			for i in rows:
				for j in columns:
					self.map[i][j] = -1 if k == '0' else 0
			self.erro = 0
		else:
			print('Erro! Invalidy entry!')
			self.erro = 1

#Initializing the mobile obstacles randomly, it receives the number of the mobile obstacles.
	def mob_obstacles(self, num):
#map_filling represents the amount of mobile obstacles already filled, obviously, it starts in 0.
		map_filling = 0
#the loop stops when it filled the map with the amount of mobile obstacles already defined.
		while map_filling < num:
#Choosing a position belonging to the map at random.
			position = []
			position.append(int(np.random.randint(0, self.row -1, size=1)))
			position.append(int(np.random.randint(0, self.column -1, size=1)))
#Checking if the position is not longer occupied with any obstacle.
			if self.map[position[0]][position[1]] != -1:
#If it is free, then we add the position in the mobile_obstacles, also we change the map and sum +1 in the map_filling.
				self.mobile_obstacles.append(position)
				self.map[position[0]][position[1]] = -1
				map_filling += 1

#In this method, it moves the mobile obstacles.
	def updating_mob_obstacles(self):
#Firstly, it checks if it exists mobile obstacles to moving, else, it does not do anything.
		if len(self.mobile_obstacles)>0:
#It stores the mobile obstacles (before moving).
			last_positions = self.mobile_obstacles
#It stores the new positions of the mobile obstacles (moving).
			new_positions = []
#It takes the current position of the mobile obstacles
			for position in last_positions:
#It creates a loop stop criterion
				stop = False
				while stop == False:
#Randomly, it chooses if the mobile obstacle should ramain in the current position or move to right, left, up and down.
#It creates a list with the all possibilities, then it choose a random element of this list.
					indexes = list(zip([-1,1,0,0,0],[0,0,0,-1,1]))
					random_index = indexes[np.random.choice(range(5))]
#If random_index == (0,0) indicates that it should remain in the current posiition and, it should stop the loop.
					if random_index == (0,0):
						new_positions.append(position)
						stop = True
					else:
#If the random_index is different of (0,0), then it check if the new position exists in the map, then it check if the position is free, if yes,
#it removes the obstacle in the old position, also, it adds the new position in the map, it stores all in the new_position list. In addition, it
# stops the loop.
						if 0<= position[0]+random_index[0] < self.row and 0<= position[1]+random_index[1]<self.column:
							if self.map[position[0]+random_index[0]][position[1]+random_index[1]] != -1:
								self.map[position[0]][position[1]] = 0
								self.map[position[0]+random_index[0]][position[1]+random_index[1]] = -1
								new_positions.append([position[0]+random_index[0], position[1]+random_index[1]])
								stop = True
#Changing the positions of the mobile obstacles with the stored information.
			self.mobile_obstacles = new_positions

#General plotting method
	def plotEnv (self, name):
#Adjusting figure size and change the title.
		plt.figure(figsize=(16,7.5))
		plt.get_current_fig_manager().set_window_title('The {} map' .format(name))
#Adding labels to x and y axis.
		plt.xlabel("ROWS", size = 14)
		plt.ylabel("COLUMNS", size= 14)
#Addding plot title.
		plt.title('The {} map' .format(name), loc='center', size=22, color = "black")
#Adjusting the ticks on both x and y axis.
		plt.xticks([i for i in range(self.row)], size=11, color = "red")
		plt.yticks([i for i in range(self.column)], size=11, color = "red")
#In the .imshow can visualize the 'bits' of our map (i.e., data matrix), also it has a range of parameters that we can use for adjusting the
#2D grid visualization (e.g., "alpha" and cmap), specifically, "alpha" "alpha" result in defined transparency and cmap" allows for defining
#a defined coloring map. Defining color map, using ListedColormap method from the colors package.
		plt.imshow(self.map, cmap = colors.ListedColormap(["darkblue","lightblue"]), alpha = 0.75)
#Creating the colorbar and adjusting it.
		plt.colorbar(format='', ticks = []).set_label('Occupied\Free')
#Displaying it.
		plt.show()

#Using the last method
	def  plotRealEnv(self):
		self.plotEnv('environment')