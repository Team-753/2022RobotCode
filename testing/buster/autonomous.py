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
        "yOffset": -60,
        "totalLength": 120
    },
    "controlPoints": [
        {
        "x": 0,
        "y": 0,
        "theta": 0,
        "d": 0,
        "speed": 1,
        "heading": 0,
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
        points = []
        for point in self.unParsedPath["controlPoints"]:
            points.append(ControlPoint(point["x"], point["y"], point["theta"], point["d"], point["speed"], point["heading"], point["stop"], point["actions"]))
        self.parsedPath = CubicBSpline(points, self.unParsedPath["initialization"]["totalLength"])
    
    def loadPath(self, pathName):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, f"paths/{pathName}.json")
        with open (filePath, "r") as f1:
            f2 = json.load(f1)
        return f2
    
    def convertAngle(self, angle):
        angle %= 360
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        return angle # is this still in unit circle degrees?
    
    def periodic(self, navxAngle, navxXDisplacement, navxYDisplacement):
        xDisplacement = navxXDisplacement + self.xOffset
        yDisplacement = navxYDisplacement + self.yOffset
        angle = self.convertAngle(navxAngle)
        
class ControlPoint:
    def __init__(self, x, y, theta, d, speed, heading, stop, actions):
        self.x = x
        self.y = y
        self.theta = theta
        self.d = d
        self.speed = speed
        self.heading = heading
        self.stop = stop
        self.actions = actions

        self.subPoint1X = -1*self.d*math.cos(self.theta) + self.x
        self.subPoint1Y = -1*self.d*math.sin(self.theta) + self.y
        self.subPoint2X = self.d*math.cos(self.theta) + self.x
        self.subPoint2Y = self.d*math.sin(self.theta) + self.y

    def updateSubPoints(self):
        self.subPoint1X = -1*self.d*math.cos(self.theta) + self.x
        self.subPoint1Y = -1*self.d*math.sin(self.theta) + self.y
        self.subPoint2X = self.d*math.cos(self.theta) + self.x
        self.subPoint2Y = self.d*math.sin(self.theta) + self.y

class CubicBSpline:
    def __init__(self, points, numberOfPoints):
        self.points = points
        self.interpolatedList = []
        for i in range(0,numberOfPoints):
            x, y = self.getPathPosition(i*(len(self.points)-1)/numberOfPoints)
            stop = False
            if i == numberOfPoints:
                stop = True
            point = {
                "coordinates": {
                    "x":x,
                    "y":y
                },
                "stop": stop, # lol fix this
                "rotation": 0, #temporary
                "wait": False, # temporary
                "speedFactor": 0.25 # very temporary
            }
            self.interpolatedList.append(point)
        return(self.interpolatedList)

    def getPathPosition(self, t):
        if t > (len(self.points) - 1):
            return(None)
        startingPoint = int(t)

        t = t % 1

        P0 = self.points[startingPoint]
        P3 = self.points[startingPoint + 1]
        P1 = [P0.subPoint2X, P0.subPoint2Y]
        P2 = [P3.subPoint1X, P3.subPoint1Y]
        P0 = [self.points[startingPoint].x, self.points[startingPoint].y]
        P3 = [self.points[startingPoint + 1].x, self.points[startingPoint + 1].y]

        a = (1-t)**3
        b = 3*((1-t)**2)*(t)
        c = 3*(1-t)*((t)**2)
        d = (t)**3

        x = a*P0[0] + b*P1[0] + c*P2[0] + d*P3[0]
        y = a*P0[1] + b*P1[1] + c*P2[1] + d*P3[1]

        return x, y