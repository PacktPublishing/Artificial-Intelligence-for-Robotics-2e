# -*- coding: utf-8 -*-
"""
Created on Sat Jul 28 21:45:53 2018

@author: Francis
"""

#Here is some Python code to illustrate how the A* algorithm works:
#We keep a set of all the grid squares on the map we have computed values for.  We’ll call this the exploredMap.
#Our map grid square object looks like this:
# globals
mapLength = 1280
mapWidth = 1200
mapSize = mapLength*mapWidth
map = []
for ii in range(0, mapWidth):
	for jj in range(0,mapLength):
		mapSq = mapGridSquare()
		mapSq.position = [ii,jj]
		mapSq.sType =EMPTY
# create obstacles
obstacles = [[1,1],[1,2],[1,3],[45,18],[32,15] …..[1000,233]]
# iterate through obstacles and mark on the map
for pos in obstacles:
	map[pos]. sType = OBSTACLE
pathGrid = []
START = [322, 128]
GOAL = [938,523]

def mapGridSquare():
	def __init__(self):
		self.F_value = 0.0  #total of G and H 
		self.G_value = 0.0  # distance to start
		self.H_value = 0.0  # distance to goal
		self.position=[0,0]   # grid location x and y
		self. predecessor =None   # pointer to previous square
		self.sType = PATH
	def compute(self, goal, start):
		self.G_value = distance(goal.position,self.position)
		self.H_value = distance(start.position,self.position
		self.F_value = self.G_value + self.H_value
		return self.F_value
def reconstructPath(current):
	totalPath=[current]
	done=False
	while not done:
		a_square = current.predecessor
		if a_square == None:  # at start position?
			done = True
		totalPath.append(a_square)
		current = a_square
	return totalPath
def A_Star_navigation(start, goal, exploredMap, map):
    while len(exploredMap>0):
    	current = findMin(exploredMap)  # the findMin function returns the grid block data with the lowest “F” score
    	if current.position == goal.position:
    		# we are done – we are at the goal
    		return reconstructPath(current)
    	neighbors = getNeighbors(current)  
    # returns all the neighbors of the current square that are not marked as obstacles
    	for a_square in neighbors:
    		if a_square.predecessor == None:
    			# square has not been previously evaluated
    			old_score = a_square.F_value
    score = a_square.compute(GOAL, START)
    # we are looking for a square that is closer to the goal than our current position
    if a_square.G_value < current.G_value:
    	a_square.predecessor = current
    	current = a_square
    	current.compute(GOAL, START)
       exploredMap.append(current)
