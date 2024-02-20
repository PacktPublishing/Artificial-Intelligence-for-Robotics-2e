# -*- coding: utf-8 -*-

#
# decision tree classifier 
# one hot encoding with "Gini" criteria
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

del toyData["Toy Name"]   # we don't need this for now

textCols = ['Color','Soft','Material']
toyData = pd.get_dummies(toyData,columns=textCols)
print toyData

dTree = tree.DecisionTreeClassifier(criterion ="entropy")
dataValues=toyData.values[:,1:]
classValues = toyData.values[:,0]
lencoder = preproc.LabelEncoder()
lencoder.fit(classValues)
classes = lencoder.transform(classValues)
classValues = list(sorted(set(classValues)))

print ""
dTree = dTree.fit(dataValues,classes)

c_data=tree.export_graphviz(dTree,out_file=None,feature_names=toyData.columns,
                             class_names=classValues, filled = True,
                             rounded=True,special_characters=True)
graph = graphviz.Source(c_data)
graph.render("toy_graph_onehot_entropy")