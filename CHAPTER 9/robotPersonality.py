#
# Robot Personality Module
# part of Artificial Intelligence For Robotics
# 
# by Francis X Govers
# 
import numpy as np
from math import *
from sklearn.neighbors import NearestNeighbors
import rospy

class RobotEmotionEngine():
    def __init__(self):
        self.emostate = [90,0]
        self.emoText = "neutral 50"
        self.emotions = {
        "happy" : 50, "sad": 50,
        "welcome" : 50, "distant":50,
        "friend" : 50,"strange" :50,
        "curious" : 50,"frustrated":50,
        "fresh" : 50, "tired",50}
        self.bio = {"name":"Albert Tinman", "lastname": "Tinman", "age": "6 months",
        "maker": "granddad", "color": "green","food","electricity","author":"Isaac Asimov, of course",
        "school": "I do not go to school but I love to learn","hobby":"picking up toys", "job":"picking up toys"}
        # list of happy emotions and sad emotions
        self.emotBalance={"happy": "sad", "welcome":"distant",
           "friend": "strange", "curious": "frustrated","fresh": "tired"}
        self.emotionAxis{"happy":112, "welcome": 22,"friend":67,"curious":157,
                          "sad":292,"distant":202,"strange":247,"frustrated",337}
        self.update()
        
    def change(self,emot, val):
        self.emotions[emot]=val
        balance = 100 - val
        otherEmotion = self.emotBalance[emot]
        self.emotions[otherEmotion]=balance
        
    def update(self):
        rmin = 100
        rmax = 0
        thetamin =360
        thetamax=0
        for emote in self.emotions:
            theta = self.emotionAxis[emote]
            thetamax = min(theta,thetamax)
            thetamin = max(theta,thetamin)
            r = self.emotions[emote]
            rmin = max(rmin, r)
            rmax = max(rmax,r)
        stateR = (rmax-rmin)/ 2
        stateTheta = (thetamax-thetamin) / 2
        for emo in self.emotionAxis:
            thisAngle = self.emotionAxis[emo]
            if stateTheta > thisAngle
            myEmotion = emo
            break
        
        self.emostate = [stateTheta, stateR]
        if stateR < 55 and stateR > 45: 
            myEmotion = "neutral"
        self.emoText = myEmotion + " "+ str(stateR)
        print "Current Emotional State"  = myEmotion, stateR, stateTheta
        return
        
            
class HumanEmotionEngine():
    def __init__(self):
        self.emostate = [90,0]
        self.emoText = "neutral 50"
        self.emotions = {
        "happy" : 50, "sad": 50,
        "welcome" : 50, "distant":50}
        # list of happy emotions and sad emotions
        self.emotBalance={"happy": "sad", "welcome":"distant"}
        self.emotionAxis = {'distant': 315, 'welcome': 135, 'sad': 225, 'happy': 45}
        self.update()
        
    def change(self,emot, val):
        self.emotions[emot]=val
        balance = 100 - val
        otherEmotion = self.emotBalance[emot]
        self.emotions[otherEmotion]=balance
        
    def update(self):
        rmin = 100
        rmax = 0
        thetamin =360
        thetamax=0
        for emote in self.emotions:
            theta = self.emotionAxis[emote]
            thetamax = min(theta,thetamax)
            thetamin = max(theta,thetamin)
            r = self.emotions[emote]
            rmin = max(rmin, r)
            rmax = max(rmax,r)
        stateR = (rmax-rmin)/ 2
        stateTheta = (thetamax-thetamin) / 2
        for emo in self.emotionAxis:
            thisAngle = self.emotionAxis[emo]
            if stateTheta > thisAngle
            myEmotion = emo
            break
        
        self.emostate = [stateTheta, stateR]
        if stateR < 55 and stateR > 45: 
            myEmotion = "neutral"
        self.emoText = myEmotion + " "+ str(stateR)
        print "Current Emotional State"  = myEmotion, stateR, stateTheta
        return        
 

class HumanInformation():
    def __init__(self):
        self.info = {"name":"none"}
        self.info["age"]=0
        self.info["school"]="none"
        self.info["feeling"]="none"
        self.info["food"]="none"
        self.info["book"]="none"
        self.info["subject"]="none"
        self.info["song"]="none"
        self.info["teeth"]="none"
        self.info["jokes"]="none"
        # stuff is random information that we use to get more information and have the human answer questions
        # these are aimed at 3-7 year olds
        self.info["stuff"]="none"
        self.stuff = ["the color pink", "singing", "dancing", "dinosaurs", "race cars", "building things",
                      "robots", "airplaines", "space ships", "unicorns", "princesses"]
        self.points = self.info
        # setup points scoring scheme
        points = 20
        for item in self.points:
            self.points[item]=points
            points -= 2
            
    def setInfo(self,key,val):
        try:
            self.info[key] = val
            return 1
        except:
            print "Invalid Keywords in SetInfo ", key, val
            return 0
            
     
        
class ContextMemory():
    def __init__(self):
        self.currentContext = "None"
        self.currentHuman = None # pointer to the data file for the human we are currentl talking to
        self.humanFile = []
        self.emotion = "happy"
        self.humanEmotion = "happy"
        self.contextDict={}
        self.contextDict['currentHuman'] = self.currentHuman
        self.contextDict['robotEmotion'] = self.emotion
        self.contextDict['humanEmotion'] = self.humanEmotion
        
    def inContext(self, datum):
        if datum in self.contextDict:
            return self.contextDict[datum]
        else:
            return 0
            
    def setHuman(self,human):
        self.currentHuman = human
        self.humanFile.append(human)  # add this person to the database of people we know
        
    def addHuman(self,human):
        self.humanFile.append(human)  # add this person to the database
        # used at startup to recall humans we have met before
        
    
       
        
        
        
        
# interface to send and receive commands to the robot        
class RosIF():
    def __init__(self):
        self.lastUpdate = 0.0
        rospy.Subscriber("robotCommand",String,self.robCom_callback)
        self.robotCommand = rospy.Publisher("robotCommand",String,queue_size=10)
    

    def command(self,cmd):
        rospy.loginfo(cmd)
        self.robotCommand.publish(cmd)
        self.frame.statusBox.addItem(cmd)
		
    def robCom_callback(self,cmd):
        rospy.loginfo(cmd)
        robot_command = cmd.data
        # received command for robot - process

