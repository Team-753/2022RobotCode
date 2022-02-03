import json
import math
import os
from hal import initialize
import wpilib
import wpilib.controller

path = {
    "initialization": {
        "angle": 60,
        "xOffset": -72,
        "yOffset": -60
    },
    "controlPoints": [
        {
        "x": 0,
        "y": 0,
        "theta": 0,
        "d": 0,
        "speed": 1,
        "stop": False,
        "actions": {}
        }
    ]
}

class Autonomous:
    def __init__(self, autonomousPathName):
        self.unParsedPath = self.loadPath(autonomousPathName)
        self.initialAngle = self.unParsedPath["initialization"]["angle"]
        self.xOffset = self.unParsedPath["initialization"]["xOffset"]
        self.yOffset = self.unParsedPath["initialization"]["yOffset"]
    
    def loadPath(self, pathName):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, f"paths/{pathName}.json")
        with open (filePath, "r") as f1:
            f2 = json.load(f1)
        return f2