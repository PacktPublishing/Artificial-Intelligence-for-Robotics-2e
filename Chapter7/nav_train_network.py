# -*- coding: utf-8 -*-
"""
Created on Tue Jul 10 17:08:50 2018

@author: bh47612
"""

# import the necessary packages
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import Adam
from sklearn.model_selection import train_test_split
from keras.preprocessing.image import img_to_array
from keras.utils import to_categorical
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os

from keras.models import Sequential
from keras.layers.convolutional import Conv2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers.core import Activation
from keras.layers.core import Flatten
from keras.layers.core import Dense
from keras import backend as K

class ConvNet():
    @staticmethod
    def create(width, height, depth, classes):
        # initialize the network
        network = Sequential()
        inputShape = (height, width, depth)


        # first set of CONV => RELU => POOL layers
        network.add(Conv2D(50, (10, 10), padding="same",
            input_shape=inputShape))
        network.add(Activation("relu"))
        network.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        # second set of CONV => RELU => POOL layers
        network.add(Conv2D(50, (5, 5), padding="same"))
        network.add(Activation("relu"))
        network.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        # third set of CONV => RELU => POOL layers
        network.add(Conv2D(50, (5, 5), padding="same"))
        network.add(Activation("relu"))
        network.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        #      Fully connected ReLU layers
        network.add(Flatten())
        network.add(Dense(500))
        network.add(Activation("relu"))

        network.add(Dense(500))
        network.add(Activation("relu"))

        # softmax classifier
        network.add(Dense(classes))
        network.add(Activation("softmax"))

        # return the constructed network architecture
        return network
    
EPOCHS = 25
LEARN_RATE = 1e-3
BATCH = 2  # batch size - modify if you run out of memory

print ("Loading Images")
images=[]
labels=[]

#location of your images
imgPath ="C:/Users/Francis/Documents/BOOK/ARTIFICIAL_INTELL_FOR_ROBOTS/CHAPTER 7/train/"
imageDirs=["left/","right/","center/"]

for imgDir in imageDirs:
   fullPath = imgPath + imgDir
    # find all the images in this directory
   allFileNames = os.listdir(fullPath)
   ifiles=[]
   label = imageDirs.index(imgDir) # use the integer version of the label
   # 0= left, 1 = right, 2 = center
   for fname in allFileNames:
       if ".jpg" in fname:
           ifiles.append(fullPath+fname)
           print("Image File:",fname)
           
   # process all of the images         
   for ifname in ifiles:
    	# load the image, pre-process it, and store it in the data list
        image = cv2.imread(ifname)
        print("Procesing ",ifname)
        # let's get the image to a known size regardless of what was collected
        image = cv2.resize(image, (800, 600))
        halfImage = 800*300  # half the pixels
        # cut the image in half -we take the top half
        image = image[0:halfImage]
        #size the image to what we want to put into the neural network
        image=cv2.resize(image,(224,224))
        # convert to grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #equalize the image to use the full range from 0 to 255
        # this gets rid of a lot of illumination variation
        image = cv2.equalizeHist(image)
        # gaussian blur the image to remove high freqency noise
        # we use a 5x kernel 
        image = cv2.GaussianBlur(image,(5,5),0)
        # convert to a numpy array
        image = img_to_array(image)
        # normalize the data to be from 0 to 1
        
        images.append(image)
        labels.append(label)

        
images = np.array(images, dtype="float") / 255.0
labels = np.array(labels)# convert to array

# split data into testing data and training data 80/20
(trainData, testData, trainLabel, testLabel) = train_test_split(images,
	labels, test_size=0.20, random_state=42)

# convert the labels from integers to vectors
trainLabel = to_categorical(trainLabel, num_classes=3)
testLabel = to_categorical(testLabel, num_classes=3)
print ("len of trainData :",len(trainData),len(trainData[0]))
# initialize the artificial neural network
print("compiling CNN...")
cnn = ConvNet.create(width=224, height=224, depth=1, classes=3)
opt = Adam(lr=LEARN_RATE, decay=LEARN_RATE / EPOCHS)
cnn.compile(loss="categorical_crossentropy", optimizer=opt,
	metrics=["accuracy"])

# train the network
print("Training network.  This will take a while")
stepsPerEpoch= int(len(trainData)//BATCH)
print ("Steps Per Epoch", stepsPerEpoch)
stepsPerEpoch = 1
trainedNetwork = cnn.fit(trainData, trainLabel,validation_data=(testData, testLabel),
	epochs=EPOCHS, verbose=1, batch_size=2)
# trainedNetwork = cnn.fit_generator(,
	# validation_data=(testImage, testLable), steps_per_epoch=len(trainImage) // BATCH,
	# epochs=EPOCHS, verbose=1)

# save the model to disk
print("Writing network to disk")

cnn.save("nav_model")

N = EPOCHS
plt.plot(np.arange(0, N), trainedNetwork.history["loss"], label="Training Loss")
plt.plot(np.arange(0, N), trainedNetwork.history["val_loss"], label="Validation Loss")
plt.plot(np.arange(0, N), trainedNetwork.history["acc"], label="Training Accuracy")
plt.plot(np.arange(0, N), trainedNetwork.history["val_acc"], label="Validation Accuracy")
plt.title("Navigation Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss - Accuracy")
plt.legend(loc="best")
#plt.savefig(args["LearningPlot-Navigation"])
plt.show()
