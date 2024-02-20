# -*- coding: utf-8 -*-

#
# decision tree classifier 
# author: Francis X Govers III
# 
# example from book "Artificial Intelligece for Roboics"
#
from sklearn import tree
import numpy as np
import pandas as pd
#from sklearn.cross_validation import train_test_split
from sklearn.metrics import accuracy_score
import sklearn.preprocessing as preproc
import graphviz

toyData = pd.read_csv("toy_classifier_tree.csv")
print ("Data length ",len(toyData))
print ("Data Shape ",toyData.shape)
dTree = tree.DecisionTreeClassifier(criterion ="gini")
dataValues=toyData.values[:,1:10]
classValues = toyData.values[:,0]
lencoder = preproc.LabelEncoder()
lencoder.fit(classValues)
classes = lencoder.transform(classValues)
classValues = list(sorted(set(classValues)))
newData = []
for ii in range(len(dataValues[0])):
    line = dataValues[:,ii]
    if type(line[0])==str:
        lencoder.fit(line)
        line = lencoder.transform(line)
    newData.append(line)
newDataArray = np.asarray(newData)
newDataArray = np.transpose(newDataArray)
dTree = dTree.fit(newDataArray,classes)

print dTree

c_data=tree.export_graphviz(dTree,out_file=None,feature_names=toyData.columns,
                             class_names=classValues, filled = True,
                             rounded=True,special_characters=True)
graph = graphviz.Source(c_data)
graph.render("toy_graph_gini")