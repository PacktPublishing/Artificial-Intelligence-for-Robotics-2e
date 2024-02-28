# -*- coding: utf-8 -*-
"""
Created on Wed Jul 11 00:08:14 2018

@author: bh47612
"""
# navigation program
# using neural network with ROS interface


import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

from keras.preprocessing.image import img_to_array
from keras.models import load_model
import numpy as np

class ROSIF():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
        self.cmd_sub = rospy.Subscriber("syscommand",String,self.cmdCallback)
        self.cmd_pub = rospy.Publisher("syscommand",String,queue_size=10)
        self.twist_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.newImage = False
        self.cmdReceived=""
        
    def callback(self):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.newImage = True
        except CvBridgeError as e:
            print(e)
    def cmdCallback(self,data):
        # receieve a message on syscommand
        self.cmdReceived = data.data  
        
        
    def getCmd(self):
        cmd = self.cmdReceived
        self.cmdReceived = ""  # clear the command so we dont do it twice
        return cmd
        
    def getImage(self):
        if self.newImage=True:
            self.newImage = False # reset the flag so we don't process twice
            return self.image
        self.newImage = False
            # we send back a list with zero elements
            img = []
            return img
       # publishing commands back to the robot 
    def pubCmd(self,cmdstr):
        self.cmd_pub.publish(String(cmdstr)):
            
    def pubTwist(self,cmd):
        if cmd == 0:  # turn left
            turn = -2
            speed = 1
        if cmd==1:
            turn = 2
            speed = 1
        if cmd ==3:
            turn=0
            speed = 1
        # all stop
        if cmd==4:
            turn = 0
            speed = 0
        cmdTwist = Twist()
        cmdTwist.linear.x = speed
        cmdTwist.angular.z = turn
        self.twist_pub.publish(cmdTwist)
            
 
def processImage(img):
    # need to process the image
    image = cv2.resize(image, (640, 480))
    halfImage = 640*240  # half the pixels
    # cut the image in half -we take the top half
    image = image[0:halfimage]
    #size the image to what we want to put into the neural network
    image=cv2.resize(image,(224,224))
    # convert to grayscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #equalize the image to use the full range from 0 to 255
    # this gets rid of a lot of illumination variation
    image = cv2.equalizeHist(image)
    # gaussian blur the image to remove high freqency noise
    # we use a 5x kernel 
    image = cv2.GaussianBlur(img,(5,5),0)
    # convert to a numpy array
    image = img_to_array(image)
    # normalize the data to be from 0 to 1
    image2 = np.array(image, dtype="float") / 255.0
    return image2
       
# MAIN PROGRAM
ic = image_converter()
rosif = ROSIF()
rospy.init_node('ROS_cnn_nav', anonymous=True)
mode = "OFF"
# load the model for regular navigation
navModel = load_model("nav_model")
toyboxModel = load_model("toybox_model")


while not rospy.is_shutdown():
    rospy.spin()
    # this mode is autonomous navigation mode
    #
    if rosif.cmdReceived=="CNN_NAV_GO":
        mode ="NAV"
    #
    # drive to the toy box
    #
    if rosif.cmdReceived=="CNN_NAV_TOYBOX":
        mode = "TOYBOX"
    #
    #stop moving! 
    #
    if rosif.cmdReceived=="CNN_NAV_STOP" or rosif.cmdReceived=="STOP":
        mode = "OFF"
        rosif.pubTwist(4)
    if mode = "NAV":
        # we feed the image data to the CNN
        image = rosif.getImage()
        if len(image)> 0:
            # classify image
            result = navmodel.predict_classes(image)
            cmd = result[0]
            rosif.pubTwist(cmd)
    if mode = "TOYBOX":
        # we feed the image data to the CNN
        image = rosif.getImage()
        if len(image)> 0:
            # classify image
            result = toyboxModel.predict_classes(image)
            cmd = result[0]
            rosif.pubTwist(cmd)  
            
    time.sleep(0.05)  # wait for next image
    
            
    
    