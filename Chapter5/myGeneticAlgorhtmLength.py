# -*- coding: utf-8 -*-
"""
Created on Thu May 31 10:43:42 2018

@author: bh47612
"""
import numpy as np
from math import *
import matplotlib.pyplot as mp
import random
import time
import pickle

# functions and classes
# action matrix - all possible combinations of actions of the three motors
ACTIONMAT = np.array([[0,0,-1],[0,0,0],[0,0,1],
       [0,-1,-1],[0,-1,0],[0,-1,1],
       [0,1,-1],[0,1,0],[0,1,1],
       [-1,0,-1],[-1,0,0],[-1,0,1],
       [-1,-1,-1],[-1,-1,0],[-1,-1,1],
       [-1,1,-1],[-1,1,0],[-1,1,1],
       [1,0,-1],[1,0,0],[1,0,1],
       [1,-1,-1],[1,-1,0],[1,-1,1],
       [1,1,-1],[1,1,0],[1,1,1]])

STATEINDEX=[]
MAXSTATE = 0

COLLISIONVOLUMES=[[0,0,95,180],[0,0,60,130]]

class RobotArm():
    def __init__(self):
        self.state = [0,0,0]
        
    def setState(self,st):
        self.state = st
        self.position = calHandPosition(st)
        
    def setGoal(self,newGoal):
        self.goal = newGoal
        
    def calcReward(self):
        dx = self.goal[0] - self.position[0]
        dy = self.goal[1] - self.position[1]
        dist2goal = sqrt(dx*dx + dy*dy)
        self.dist2goal = dist2goal
        # we want the reqard to be 100 if the goal is met
        # and porportional to the distance from goal otherwise
        # the arm is 340mm long, so that is as far away as we can get
        #
        reward = (340.0-dist2goal)/340.0 * 100.0
        # check for collisions with robot body or the floor
        colide = isWithin(self.position)
        if colide:
            reward=0
            #print ("COLLISION WARNING")
        self.reward = reward
        return reward
        

    def step(self,act,learningRate):
        newState = self.state + (act * learningRate)
        #newState=np.array([roundup(newState[0]),roundup(newState[1]),roundup(newState[2])])
        # range check
        for ii in range(3):
            newState[ii]=max(newState[ii],0)
            newState[ii]=min(newState[ii],255.0)
        
        self.setState(newState)
        reward = self.calcReward()
        
        return self.state,reward
# for a given action, return the new state 

def isBetween(pnt,minpt,maxpt):
    betw = False
    if pnt > minpt and pt<maxpt:
        betw == True
    return betw
        
def isWithin(point):
    # is this point within the given area
    # used for collision detection
    collision=False
    for area in COLLISIONVOLUMES:
        x1,y1,x2,y2=area
        x,y = point
        if x < x2 and x > x1 and y<y2 and y>y1:
            # yes, point is in volume
            collision=True
    return collision
   
def predictReward(state,goal,action, learningRate):
    newState = state + (action * learningRate)
    position = calHandPosition(newState)
    dx = goal[0] - position[0]
    dy = goal[1] - position[1]
    dist2goal = sqrt(dx*dx + dy*dy)
    dist2goal = dist2goal
    # we want the reqard to be 100 if the goal is met
    # and porportional to the distance from goal otherwise
    # the arm is 340mm long, so that is as far away as we can get
    #
    reward = (340.0-dist2goal)/340.0 * 100.0
    #reward = (120.0-dist2goal)/120.0 * 100.0
    # check for collisions with robot body or the floor
    colide = isWithin(position)
    if colide:
        reward=0
        #print ("COLLISION WARNING")
    return reward,newState
# just a utility to display the joint angle in degrees
def joint2deg(jointPos):
    return jointPos * (180.0 / 255.0)

def calHandPosition(state):

    m1,m2,m3=state
    # calculate hand position based on the position of the servo motors
    # m1, m2, m3 = motor command from 0 to 255
    # forward kinematics
    # we first convert each to an angle 
    d1 = 102.5  # length of first joint (sholder to elbow) in mm
    d2 = 97.26  # length of second joint arm (elbow to wrist) in mm
    d3 = 141    # length of thrird joint arm (wrist to hand)
    right = pi/2.0 # right angle, 90 degrees or pi/2 radians
    # convert angle from units to radians and reverse the direction
    m1Theta = pi - m1*(pi/255.0)
    m2Theta = pi - m2*(pi/255.0)
    m3Theta = pi - m3*(pi/255.0)
    
    # add the joint angles  m2 = m1 + m2, m3 = m2+m3
    m2Theta = m1Theta-right+m2Theta
    m3Theta = m2Theta-right+m3Theta
    # compute the cartesian coordinates of the arm in mm
    # translate the origin of the new joint to the end of the preceeding joint
    joint1 = np.array([d1*cos(m1Theta),d1*sin(m1Theta)])
    joint2 = np.array([d2*cos(m2Theta),d2*sin(m2Theta)])+joint1
    joint3 = np.array([d3*cos(m3Theta),d3*sin(m3Theta)])+joint2
    return joint3

def randint(num):
    return random.randint(0,num)

def computeFitness(population, goal, learningRate, initialPos):
    fitnessList = []
    gamma = 0.6
    state=initialPos
    index = 0
    for chrom in population:
        value=[0]
        snip = 0
        state=initialPos
        for allele in chrom:
            action = ACTIONMAT[allele]
            indivFit, state = predictReward(state,goal,action,learningRate)
            value.append(indivFit)
            if indivFit > 98:
                fitnessList.append([indivFit,index])
            # we are at the goal - snip the DNA here
                oldLen = len(chrom)
                chrom=chrom[0:snip+1]
                population[index]=chrom
                #print ("snipped chrom ",oldLen,len(chrom))
                break
            snip+=1
        # take the maximum value in the chromosome    
        # penalize for longer chromosomes
        valueNum=max(value)-(len(chrom)/2)
        fitnessList.append([valueNum,index])
        index += 1
    return fitnessList
    
    
def make_new_individual():
     # individual length of steps
    lenInd = random.randint(10,60)
    chrom = [] # chromosome description
    for ii in range(lenInd):
        chrom.append(randint(26))
    return chrom
# select an individual in purportion to its value
 # i.e. the higher the value, the better the odds of getting picked
def roulette(items):
     total_weight = 0
     for item in items:
         total_weight= total_weight + item[0]
     weight_to_target = random.uniform(0, total_weight)
     for item in items:
         weight_to_target -= item[0]
         if weight_to_target <= 0:
             return item
# main Program
startTime = time.time()             
INITIAL_POS = [127,127,127]
GOAL=[-107.39209423,  -35.18324771]
robotArm=RobotArm()
robotArm.setGoal(GOAL)
population = 200
learningRate = 5
crossover_chance = .50
mutate_chance = .001
pop = []
for i in range(population):
    pop.append(make_new_individual())
 
trainingData=[]
epochs = 100
print ("Training Rate",learningRate," Pop: ",population)
for jj in range(epochs):
 # evaluate the population
    fitnessList = computeFitness(pop,GOAL,learningRate, INITIAL_POS)
     # sort the fitness to get the best indivuals
    fitnessList.sort(reverse=True)
    # we keep the top 50 % of the population
    fitLen = int(population/2)
    fitnessList = fitnessList[0:fitLen]
     # survival of the fittest...
     #pull out the top performer
    hoff = pop[fitnessList[0][1]]
    print("HOF = ",fitnessList[0], " length ",len(hoff))
    if fitnessList[0][0] > 99.0:
        # we are done!
        break
    trainingData.append(fitnessList[0][0])
    newPop = []
    for ddex in fitnessList:
        newPop.append(pop[ddex[1]])
    #print ("Survivors: ",len(newPop))
 
# crossover 
# pick to individuals at random
# on the basis of fitness
    numCross = population-len(newPop)-10
    #print ("New Pop Crossovers",numCross)
#   # 
    # add 5 new random individuals
    for kk in range(10):
        newPop.append(make_new_individual())
    for kk in range(int(numCross)):
        p1 = roulette(fitnessList)[1]
        p2 = roulette(fitnessList)[1]
        chrom1 = pop[p1]
        chrom2 = pop[p2]
        lenChrom = min(len(chrom1),len(chrom2))
        xover = randint(lenChrom)
        # xover is the point where the chromosomes cross over
        newChrom = chrom1[0:xover]+chrom2[xover:]
        # now we do mutation
        bitDex = 0
        for kk in range(len(newChrom)-1):
            mutDraw = random.random()
            if mutDraw < mutate_chance:
                # a mutation has occured!
                bit = randint(26)
                newChrom[kk]=bit
                #print ("mutation")
        newPop.append(newChrom)

     # welcome the new baby from parent 1 (p1) and parent 2 (p2)
    print("Generation: ",jj)
    pop=newPop
stopTime = time.time()
totTime = stopTime - startTime
print (" Total Time to Run ",totTime, "Score=",fitnessList[0][0], len(hoff)    )
mp.plot(trainingData)
mp.show()
