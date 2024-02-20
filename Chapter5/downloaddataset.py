from roboflow import Roboflow 

rf = Roboflow(api_key=”*****************”) 

project = rf.workspace(“toys”).project(“toydetector”) 

dataset = project.version(1).download(“yolov8”) 

