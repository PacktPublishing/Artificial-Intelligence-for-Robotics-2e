# -*- coding: utf-8 -*-
"""
Created on Sat Feb 03 23:50:32 2018

@author: Francis
"""

import time
from numpy import mean
import matplotlib.pyplot as plt
import math
#
# set our frame rate - how many cycles per second to run our loop?
FRAMERATE = 5
# how long does each frame take in seconds?
FRAME = 1.0/FRAMERATE
# initialize myTimer
myTimer = 0.0
# how many cycles to test?  counter*FRAME = runtime in seconds
counter =40 
# fudge factor for our timing loop computations
TIME_CORRECTION= 0.0
# place to store data
dataStore = []
# Operator information  ready to go
print "START COUNTING: FRAME TIME", FRAME, "RUN TIME:",FRAME*counter
# initialize the precision clock
myTime = topOfFrameTime = time.clock()
# save the starting time for later
masterTime=myTime
timeError=0.0
# begin our timing loop
for ii in range(counter):
    # we start our frame - this represents doing some detailed math calculations
    topOfFrameTime = time.clock()
    
    sleepTime = FRAME + timeError
    sleepTime = max(-FRAME, sleepTime)
    time.sleep(FRAME)
    # set our timer up for the next frame
    endOfFrameTime = time.clock()
    thisFrameTime = endOfFrameTime-topOfFrameTime
    timeError=FRAME-thisFrameTime
    print endOfFrameTime,topOfFrameTime,thisFrameTime,timeError
    timeError=min(FRAME,timeError)
    dataStore.append(timeError)
# Timing loop test is over - print the results
#

endTime = time.clock() - masterTime
avgTime = endTime / counter
print "FINISHED COUNTING"
print "REQUESTED FRAME TIME:",FRAME,"AVG FRAME TIME:",avgTime
print "REQUESTED TOTAL TIME:",FRAME*counter,"ACTUAL TOTAL TIME:", endTime
print "AVERAGE ERROR",FRAME-avgTime, "TOTAL_ERROR:",(FRAME*counter) - endTime
print "AVERAGE SLEEP TIME ERROR: ",mean(dataStore)
# loop is over, plot result
#n, bins, patches = plt.hist(dataStore, 50, normed=1, facecolor='green', alpha=0.75)
#plt.show()