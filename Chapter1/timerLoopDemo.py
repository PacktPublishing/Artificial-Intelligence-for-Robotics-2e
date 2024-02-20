# -*- coding: utf-8 -*-
"""
Created on Sat Feb 03 21:49:08 2018

@author: Francis Govers
# Chapter 1: Timing Loop Control Demo Program
# this program demonstrates how to control your program for "soft real time"
  by controlling the time each loop executes
"""

import time
from numpy import mean
import matplotlib.pyplot as plt
import math
#
# set our frame rate - how many cycles per second to run our loop?
FRAMERATE = 30
# how long does each frame take in seconds?
FRAME = 1.0/FRAMERATE
# initialize myTimer
myTimer = 0.0
# how many cycles to test?  counter*FRAME = runtime in seconds
counter = 2000
# fudge factor for our timing loop computations
TIME_CORRECTION= 0.0
# place to store data
dataStore = []
# Operator information  ready to go
print "START COUNTING: FRAME TIME", FRAME, "RUN TIME:",FRAME*counter
# initialize the precision clock
myTime = newTime = time.time()
# save the starting time for later
masterTime=myTime
frameKnt = 0 # frame counter
# begin our timing loop
for ii in range(counter):
    # we start our frame - this represents doing some detailed math calculations
    # this is just to burn up some CPU cycles
    for jj in range(1000):
        x = 100
        y = 23 + ii
        z = math.cos(x)
        z1 = math.sin(y)
    #
    # read the clock after all compute is done
    # this is our working frame time
    #
    frameKnt += 1  # increment frame counter
    if frameKnt % FRAMERATE == 0 : 
        print "Frame Completed",frameKnt
        
    newTime = time.time()
    # how much time has elapsed so far in this frame
    # time = UNIX clock in seconds
    # so we have to subract our starting time to get the elapsed time
    myTimer = newTime-myTime 
    # what is the time left to go in the frame?
    timeError = FRAME-myTimer
    
  
    # OK time to sleep
    # the TIME CORRECTION helps account for all of this clock reading
    # this also corrects for sleep timer errors
    # we are using a porpotional control to get the system to converge
    # if you leave the divisor out, then the system oscillates out of control
    sleepTime = timeError + (TIME_CORRECTION/2.0)
    # quick way to eliminate any negative numbers
    # which are possible due to jitter 
    # and will cause the program to crash
    sleepTime=max(sleepTime,0.0)
    # put this process to sleep
    time.sleep(sleepTime)
    #print timeError,TIME_CORRECTION
    # set our timer up for the next frame
    time2=time.time()
    measuredFrameTime = time2-myTime
    ##print measuredFrameTime,
    TIME_CORRECTION=FRAME-(measuredFrameTime)
    dataStore.append(measuredFrameTime*1000)
    #TIME_CORRECTION=max(-FRAME,TIME_CORRECTION)
    #print TIME_CORRECTION
    myTime = time.time()
    
    #print newTime,myTime,newTime-myTime

# Timing loop test is over - print the results
#

endTime = time.time() - masterTime
avgTime = endTime / counter
print "FINISHED COUNTING"
print "REQUESTED FRAME TIME:",FRAME,"AVG FRAME TIME:",avgTime
print "REQUESTED TOTAL TIME:",FRAME*counter,"ACTUAL TOTAL TIME:", endTime
print "AVERAGE ERROR",FRAME-avgTime, "TOTAL_ERROR:",(FRAME*counter) - endTime
print "AVERAGE SLEEP TIME: ",mean(dataStore),"AVERAGE RUN TIME",(FRAME*1000)-mean(dataStore)
# loop is over, plot result
n, bins, patches = plt.hist(dataStore, 50, normed=1, facecolor='green', alpha=0.75)
plt.show()
plt.plot(dataStore)
plt.show()
