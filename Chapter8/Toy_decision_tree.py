#
# decision tree classifier 
# author: Francis X Govers III
# 
# example from book "Artificial Intelligece for Roboics"
#
from sklearn import tree
import numpy as np
import pandas as pd
from sklearn.cross_validation import train_test_split
from sklearn.metric import accuracy_score

toyData = pd.read_csv("toy_classifier_tree.csv")
print "Data length ",len(toyData)
print "Data Shape ",toyData.shape
dTree = tree.DecisionTreeClassifier()
dataValues=[:,1:11]
classValues = [:,0]
dTree = dTree.fit(dataValues,classValues)

c_data=tree.export_graphviz(dTree,out_file="graph_toys")
graph = graph.graphviz.Source(c_data)
graph.render("Toys")