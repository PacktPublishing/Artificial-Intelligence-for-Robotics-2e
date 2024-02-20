# Test writing and reading a list
import pickle

myList = [[1,2,3], [4,5,6],[7,8,9]]
fileName = "testFile.txt"
fileName2 = "testPickle.txt"
print("Before")
print(myList)
myFile=open(fileName, "w")
for item in myList:
    myFile.write(str(item))
    myFile.write("\n")

myFile.close()

# read file back in
myFile=open(fileName, "r")
# open file and read the content in a list
with open(fileName, 'r') as fp:
    newArray=[]
    elem = []
    for line in fp:
        # remove linebreak from a current name
        # linebreak is the last character of each line
        x = line[:-1]
        x = x.replace("[","")
        x = x.replace("]","")
        x = x.replace(" ", "")
        y = x.split(",")
        elem=[]
        for item2 in y:
            elem.append(int(item2))
        newArray.append(elem)

   

# display list
print "AFTER:"
print(newArray)

myFile = open(fileName2,"w")
pickle.dump(myList, myFile)
print "Pickle Complete"
myFile.close
myFile = open(fileName2, "r")
myNewArray = pickle.load(myFile)
print myNewArray
print "Pickle read"