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
dataValues=toyData.values[:,:10]
classValues = toyData.values[:,0]
lencoder = preproc.LabelEncoder()
newData=[]
for ii in range(len(dataValues[0])):
    line = dataValues[:,ii]
    if type(line[0])==str:
        print "Original column from data \n",line
        # we have to do the label encoder first..
        lencoder = lencoder.fit(line)
        line1 = lencoder.transform(line)
        print "Label encoded line \n",line1
        # one hot encoder
        line1= line1.reshape(1,-1)
        ohe = preproc.OneHotEncoder()
        line = ohe.fit_transform(line1)
        #print"after OHE",line
        line2 = line.toarray()
        print"after One Hot Encode: \n",line2, line2.shape
        print line2.shape
        line=line2[0]
        
    line=line.tolist()    
    newData.append(line)

print "old shape = ",newData
newDataArray = np.array(newData)
newDataArray = np.transpose(newData)
print("New Shape= ",newDataArray.shape) 
print ""
dTree = dTree.fit(newDataArray,classValues)

c_data=tree.export_graphviz(dTree,out_file=None,feature_names=toyData.columns,
                             class_names=classValues, filled = True,
                             rounded=True,special_characters=True)
graph = graphviz.Source(c_data)
graph.render("toy_graph_gini_onehot")