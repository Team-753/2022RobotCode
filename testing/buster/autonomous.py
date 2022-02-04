import json
import math
import os
import wpilib
import wpimath.controller

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
    
    def convertAngle(self, angle: float):
        angle %= 360
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        return(angle)
    
    def convertToXY(self, xDistance, yDistance, speed):
        hypotenuse = math.hypot(xDistance, yDistance)
        ratioX = xDistance/hypotenuse
        ratioY = yDistance/hypotenuse
        x = ratioX*speed
        y = ratioY*speed
        return(x, y)
    
    def passedTargetCheck(self, currentX, currentY, nextX, nextY):
        pass
    
    
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
    def __init__(self, points, numberOfPoints, startingAngle):
        self.points = points
        self.interpolatedList = []
        self.pointIndex = 0
        self.speedThreshold = 1
        self.targetAngle = 0
        self.startAngle = startingAngle
        for i in range(0,numberOfPoints):
            x, y = self.getPathPosition(i*(len(self.points)-1)/numberOfPoints)
            point = {}
            self.interpolatedList.append(point)
        return(self.interpolatedList)

    def getPathPosition(self, t):
        point = {
            "x": 0,
            "y": 0,
            "speed": 0,
            "heading": 0,
            "stop": False,
            "actions": {}
        }
        if t > (len(self.points) - 1):
            return(None)
        if t > self.pointIndex:
            self.speedThreshold = self.points[self.pointIndex]["speed"]
            point["stop"] = self.points[self.pointIndex]["stop"]
            point["actions"] = self.points[self.pointIndex]["actions"]
            if t > 1:
                self.startAngle = self.points[self.pointIndex - 1]["heading"]
            self.targetAngle = self.points[self.pointIndex]["heading"]
            self.pointIndex += 1
        startingPoint = int(t)

        t = t % 1
        
        heading = ((self.targetAngle - self.startAngle) * t) + self.startAngle
        if heading > 180:
            heading -= 360
        elif heading < -180:
            heading += 360

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

        point["x"] = x
        point["y"] = y
        point["speed"] = self.speedThreshold
        point["heading"] = heading
        return point
