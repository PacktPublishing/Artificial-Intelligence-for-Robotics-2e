import rclpy
import time
import random
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Int32
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
import math
import pickle

global learningRate = 0.1  # learning rate

# we are rounding off aspects to the nearest quarter
def round4(x):
      return (math.round(x*4)/4)
# function to restrict a variable to a range.  if x < minx, x=min x,etc.
def rangeMinMax(x,minx,maxx):
      xx = max(minx,x)
      xx = min(maxx,xx)
      return xx

def sortByQ(listByAspect):
      return(listByAspect[2])

class LearningHand(Node):
	def __init__(self):
		super().__init__('armQLearn')   # node name
		# we need to both publish and subscribe to the RobotCmd topic
        self.armPosSub = self.create_subscription(Int32MultiArray, "xarm_pos", self.armPosCallback, 10)
		self.cmdSubscribe = self.create_subscription(String, 'RobotCmd', self.cmdCallback,10)
		self.cmdPub = self.create_publisher(String, 'RobotCmd', 10)
        self.wristPub = self.create_publisher(Int32,'xarmWrist', 10)
        # declare parameter for number of repetitions
        self.declare_parameter('ArmLearningRepeats', rclpy.Parameter.Type.INTEGER)
        # get the current value from configuration
        self.repeats = self.get_parameter('ArmLearningRepeats').get_parameter_value().int_value

        self.mode = "idle"
        self.armInterface = ArmInterface()
        # define the state space
        stateSpace = []
        # state space is the target aspect and the hand angle
        # aspect is length / width length along x axis(front back) width on y axis)
        aspects = [0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75]
        handAngles = [90, -45, 0, 45]  # note +90 and -90 are the same angle
        for jj in range(0,len(aspects)):
            for ii in range(0,4):
                stateSpace.append([aspects[jj], handAngles[ii],0.0])
                  

        # possible hand actions
        actionSpace = [-90, -45, 0, 45] # note +90/-90 is the same position
        # 

    def sndCmd(self,msgStr):
        msg = String()
        msg.data = msgStr
        self.cmpPub.publish(msg)

    def setHandAngle(self,ang):
        msg = Int32()
        msg.data = ang
        self.wristPub.publish(msg)

    def armPosCallback(self,msg):
        self.currentArmPos = msg.data

    def setActionPairs(self,pairs):
        self.stateActionPairs = pairs
        
    # when we do the training, pick an aspect and run 100 trials, then change the aspect and try again
    # do six trials 90, 45, 0, -45, -90 degrees in various orientations
    def training(self, aspect):
        # get the aspect from the vision system
        #aspect = 1.0 # start here
                
        stateActionPairs.sort(key=sortByQ)   # sort by Q value
        if len(stateActionPairs)<1:
              #error - no aspects found!
              #
            self.get_logger().error("qLearningHand No Aspect for Training")
            return
        else:
            mySetup = stateActionPairs[0] # using the highest q value
            handAngle = mySetup[1]
            myOldQ = mySetup[2]
            sndCmd("ARM MID_CARRY")
            timer.pause(1.0) 
            sndCmd("ARM GRASP")
            time.sleep(1.0)
            setHandAngle(handAngle) 
            time.sleep(0.3)
            # close the gripper
            sndCmd("ARM GRASP_CLOSE")
            time.sleep(0.5)
            # now raise the arm
            sndCmd("ARM MID_CARRY")
            time.sleep(1.0)
            #check to see if grip is OK
            handPos = self.currentArmPos[0]
            gripSuccess = False
            if handPos > 650:  ## fail
                gripSuccess = -1  # reward value of not gripping
            else:  # success!
                gripSuccess = +1  # reward value of gripping
            # adjust Q score
            # the Bellmand Equation
            ### Q(s, a) = Q(s, a) + α * [R + γ * max(Q(s', a')) - Q(s, a)]
            newQ = myOldQ + (learningRate*(gripSuccess))
            mySetup[2]=newQ
            foundStateActionPair = False
            # re insert back into q learning array
            for i in range (0,len(stateActionPairs):
                thisStateAction = stateActionPairs[i]
                if thisStateAction[0] == mySetup[0] and thisStateAction[1] == mySetup[0]:
                    foundStateActionPair=True
                    stateActionPairs[2]=mySetup[2] # store the new q value in the table
            if not foundStateActionPair:
                # we don't have this in the table - let's add it
                stateActionPairs.append(mySetup)
            

        



		
    def cmdCallBack(self,msg):
        robotCmd = msg.data
        if robotCmd == "GoLearnHand":
              self.mode = "start"
        if robotCmd == "StopLearnHand":
              self.mode = "idle"
        
    def LearningCycle(self):
        if self.mode != "start":
            return
        

class ArmInterface():
    init(self):
        self.armPosSub = self.create_subscription(Int32MultiArray, 'xarm_pos',self.armPosCallback, 10)
	    self.armAngSub = self.create_subscription(Int32MultiArray, 'xarm_angle',self.armAngCallback, 10)
        self.armPosPub = self.create_publisher(Int32MultiArray, 'xarm')
    
    def armPosCallback(self,msg):
        self.armPos = msg.data
    
    def armAngCallback(self, msg):
        self.armAngle = msg.data 
        # decoder ring: [grip, wrist angle, wrist pitch, elbow pitch, sholder pitch, sholder yaw]

    def setArmPos(self,armPosArray):
        msg = Int32MultiArray
        msg.data = armPosArray
        self.armPosPub.publish(msg)
        


### MAIN ####
# persistent training file to opeate the arm
ArmTrainingFileName = "armTrainingFile.txt"
armIf = ArmInterface()
armTrainer = LearningHand()
#open and read the file after the appending:
f = open(ArmTrainingFileName, "r")
savedActionPairs = pickle.load(f)
armTrainer.setActionPairs(savedActionPairs)
f.close()

aspectTest = [1.0, 0.5, 1.5,2]
trainingKnt = 20
for jj in aspectTest:
    for ii in range(0,trainingKnt):
        print("Starting Training on Aspect ", jj)
        armTrainer.training(jj)

f = open("ArmTrainingFileName", "w")
# open file in write mode
pickle.dump(armTrainer.stateActionPairs,f)
print("Arm Training File Written")
f.close()

