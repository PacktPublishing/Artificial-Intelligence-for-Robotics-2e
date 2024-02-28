#!/usr/bin/env python
"""
Created on Fri Feb 09 19:06:34 2018

@author: Francis
"""

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import serial

#GLOBAL VARIABLES
# set our frame rate - how many cycles per second to run our loop?
FRAMERATE = 30.0
# how long does each frame take in seconds?
FRAME = 1.0/FRAMERATE
# initialize myTimer
topOfFrame = 0.0
endOfWork = 0.0
endOfFrame=0.0
# how many cycles to test?  counter*FRAME = runtime in seconds
counter = 2000
# fudge factor for our timing loop computations
TIME_CORRECTION= 0.0

ARMPOSITIONS=[]
ap1 = [0,30,40,40,0,0]  # low carry position
# forward grab position
ap2 = []
# grasp position
ap3=[]
class RosIF():
    def __init__(self):
        self.speed = 0.0
        self.turn = 0.0
        self.lastUpdate = 0.0
	self.newCommand = False
	self.stop = False
        rospy.init_node('robotControl', anonymous=True)
        rospy.Subscriber("cmd_vel",Twist,self.cmd_vel_callback)
        rospy.Subscriber("robotCommand",String,self.robCom_callback)
        self.telem_pub = rospy.Publisher("telemetry",String,queue_size=10)
        self.robotCommand=rospy.Publisher("robotCommand",String,queue_size=10)

    def setRobot(self,robot):
	self.myRobot = robot
    
    def cmd_vel_callback(self,data):
        self.speed = int(data.linear.x*100.0)
        self.turn = int(data.angular.z*100.0)
        self.lastUpdate = time.time()
	self.newCommand = True
        
    def command(self,cmd):
        rospy.loginfo(cmd)
        self.robotCommand.publish(cmd)

    def robCom_callback(self,cmd):
        rospy.loginfo(cmd)
        robot_command = cmd.data
        # received command for robot - process
        if robot_command == "STOP":
            self.speed=0
            self.turn=0
	    self.stop = True
	    self.newCommand=True
	    
        if robot_command == "GO":
            robot.go()

# This object encapsulates our robot
class Robot():
    def __init__(self):
        # position x,y
        # velocity vx, vy
        # accelleration ax, ay
        # angular position (yaw), angular velocity, angular accelleration
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.ax = 0.0
        self.ay = 0.0
        # turn individual values into vectors
        
        self.yaw = 0.0
        self.vYaw = 0.0
        self.aYaw = 0.0
        self.vectorize()
        # status value for telemetry
        # current value for drive motors
        self.rightMotorPower=0
        self.leftMotorPower=0
        # current value for what we set the motors to (used for control)
        self.rightMotorCommand=0
        self.leftMotorCommand=0
        #
        # this describes the arm in servo motor units 
        #  from 800 to 2200 with 1500 being neutral
        self.armMotor1=0  # sholder rotate
        self.armMotor2=0  # sholder up/down
        self.armMotor3=0  # elbow up/down
        self.armMotor4=0  # wrist up/down
        self.armMotor5=0  # wrist rotate
        self.armMotor6=0  # hand open/close
        # 
        # arm position in degrees with front and horizontal being zero
        # each servo has a 165 degree range 
        self.sholderYaw = 0.0
        self.sholderPitch = 0.0
        self.elbowPitch = 0.0
        self.wristPitch = 0.0
        self.wristRotate = 0.0
        # handUnits 0 = closed 100 = open
        self.handOpenClose = 0
        # hand position in 3d cartesian coordinates
        self.handPosition = [0.0, 0.0, 0.0]
        # hand location in 3d polar coordinates (horiz angle, vert angle, distance)
        self.handPolar = [0.0,0.0,0.0]
        #
        # sonar data from our sonar sensors
        self.sonarLeft=0.0
        self.sonarCenter=0.0
        self.sonarRight=0.0
        
        # used to send commands to arudino and motors
        self.robotCommand = False   # is there a new command to send to robot?
        self.lastHeartBeat = 0.0  # time of last heartbeat from arduino
	self.stop = False # stop flag for stop command

    def stop():
	self.stop=True
	self.leftMotorCommand=0
	self.rightMotorCommand=0
        
    def armMotor2Angles(self):
        # convert arm angles from motor units to degrees
        self.sholderAngle=(self.armMotor1-1500)*(82.5/700)
        self.sholderPitch=(self.armMotor2-1500)*(82.5/700)
        self.elbowPitch=(self.armMotor3-1500)*(82.5/700)
        self.wristPitch=(self.armMotor4-1500)*(82.5/700)
        self.wristRotate=(self.armMotor5-1500)*(82.5/700)

        
    def recHeartBeat(self, hbTime):
        hbRecieved= time.time()  # 
        self.latency = hbRecieved - hbTime
        self.lastHeartBeat=hbRecieved
        
    def vectorize(self):
        # this turns position, velocity, and accel into a vector
        self.vecX = [self.x,self.vx,self.ax]
        self.vecY = [self.y,self.vy,self.ay]
        self.ang = [self.yaw,self.vYaw,self.aYaw]

def readBuffer(buff):
# the data we get is a series of lines separated by EOL symbols
# we read to a EOL character (0x10) that is a line
# process complete lines and safe partial lines for later
#    
    EOL = '\n'
    if len(buff)==0:
        return
    dataLine = ""
    lines=[]
    for inChar in buff:
        if inChar != EOL:
            dataLine +=inChar
        else:
            lines.append(dataLine)
            dataLine=""
    for telemetryData in lines:
        processData(telemetryData)
    return dataLine
        
def processData(dataLine):
    if len(dataLine)==0:
	return
    #
# take the information from the arduino and process telemetry into
# status information
# we recieve either heartbeat (HBB), TL1 (telemtry List 1), or ERR (Error messages)
# we are saving room for other telemetry lists later
    dataType = dataLine[0]
    payload = dataLine[1:]  # rest of the line is data
    if dataType == 'H':
	print "Heartbeat Received",payload
        # process heartbeat
    # we'll fill this in later
        pass
    if dataType == "T": # telemetry list 1
    # we'll add this later
        pass
    if dataType == "E":  # error message
        print "ARUDUINO ERROR MESSAGE ",payload
        # log the error
        rospy.loginfo(payload)
    return
        
            
         

# main program starts here
# *****************************************************
rosif = RosIF()  # create a ROS interface instance
robot = Robot()  # define our robot instance
serialPort = "/dev/ttyACM0"
# open serial port to arduino
ser = serial.Serial(serialPort,115200,timeout=0.0222) #
# serial port with setting 38,400 baud, 8 bits, No parity, 1 stop bit
try:
    ser.close()
    ser.open()
except:
    print "SERIAL PORT FOR ARDIONO DID NOT OPEN ", serialPort
    ser.close()
    raise

time.sleep(2)
timeError = 0.0
frameTime = time.time()
frameKnt = 0  # counter used to time processes
while not rospy.is_shutdown():
    # main loop
    topOfFrame=time.time()
    # do our work
    # read data from the seral port if there is any
    serData = ser.readline()
    if type(serData)!= bool:
	if len(serData) > 0: print "-->",serData,
    # process the data from the arduino
    # we don't want any blocking, so we use read and parse the lines ourselves
    holdBuffer = readBuffer(serData)
    #drive command
    com = ','  # comma symbol we use as a separator
    EOL = '\n'
    if rosif.stop == True:
        ardCmd = "STOP\n"
        ser.write(ardCmd)
        rosif.stop==False
    if rosif.newCommand:
	leftMotorCmd = abs(rosif.speed * 1)
	rightMotorCmd = abs(rosif.speed * 1)
	turn = (rosif.turn*1)
	leftMotorCmd += turn
	rightMotorCmd -= turn
	if rosif.speed >= 0:
		leftMotorCmd+=100;
		rightMotorCmd +=100;

        print "Motor Command ",rosif.speed, rosif.turn,leftMotorCmd,rightMotorCmd
	leftMotorCmd = min(200,leftMotorCmd)
        rightMotorCmd = min(200,rightMotorCmd)
        rightMotorCmd = max(0,rightMotorCmd)
        leftMotorCmd = max(0,leftMotorCmd)
	ardCmd = 'D'+chr(leftMotorCmd)+chr(rightMotorCmd)+EOL
        ser.write(ardCmd)
	print "cmd send ",ardCmd
        #ser.flush()  # output Now
	rosif.newCommand = False # reset command flag
    if frameKnt % (30)==0:  # twice per second
        hbMessage = 'H'+str(time.time())+EOL
        #ser.write(hbMessage)
        #ser.flush()  # output Now
	msg = "HBB "+str(time.time())
	#rosif.command(msg)
        
    frameKnt +=1
    endOfWork = time.time()
    if (endOfWork-frameTime > 1.0):
	#print "Frames per sec",frameKnt
	frameKnt = 0
	frameTime = endOfWork
    frameKnt=min(FRAMERATE,frameKnt)  # just count number of frames in a second
    
    # done with work, now we make timing to get our frame rate to come out right
    #
    endOfWork = time.time()
    workTime = endOfWork - topOfFrame
    sleepTime = (FRAME-workTime)+ timeError
    #print "Sleep Time ",sleepTime
    sleepTime = max(0,sleepTime)
    sleepTime = min(0.033,sleepTime)
    time.sleep(sleepTime)
    endOfFrame = time.time()
    actualFrameTime = endOfFrame-topOfFrame
    timeError = FRAME-actualFrameTime
    # clamp the time error to the size of the frame
    timeError = min(timeError,FRAME)
    timeError = max(timeError,-FRAME)
# end of the main loop
#
ser.close()
