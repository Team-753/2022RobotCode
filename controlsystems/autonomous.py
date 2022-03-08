import json
import math
import os
import wpilib
import wpimath.controller
import wpimath.trajectory

class Autonomous:
    def __init__(self, autonomousPathName, nvxObj):
        self.navx = nvxObj
        self.waiting = 0
        unParsedPath = self.loadPath(autonomousPathName)
        self.navx.setAngleAdjustment(unParsedPath[0]["heading"])
        self.xOffset = unParsedPath[0]["x"]
        self.yOffset = unParsedPath[0]["y"]
        points = []
        maxAngularVelocity = math.pi / 2 # change to degrees
        maxAngularAcceleration = math.pi # change to degrees
        for point in unParsedPath:
            points.append(ControlPoint(point["x"], point["y"], point["theta"], point["d"], point["speed"], point["heading"], point["stop"], point["actions"]))
        length = self.computeRelativeLength(points)
        self.generatedPath = self.generatePath(points, length) # NOTE: 500 is temporary
        self.headingController = wpimath.controller.ProfiledPIDController(0.005, 0.0025, 0, 
        wpimath.trajectory.TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)) # needs some testing
        self.headingController.enableContinuousInput(-math.pi, math.pi)
        self.speedController = wpimath.controller.PIDController(0.0025, 0.001, 0) # god this needs sooo much testing
        self.OGRemainder = 0
        self.pathPosition = 1
        for idx, generatedPoint in enumerate(self.generatedPath):
            if idx == len(self.generatedPath):
                break
            if generatedPoint["stop"]:
                endPoint = idx
        self.calculateRemainder(0, endPoint)
        self.OGRemainder = self.pathRemainder
        self.Timer = wpilib.Timer()
    
    def calculateRemainder(self, startPoint, endPoint):
        self.pathRemainder = 0
        self.previousRemainder = math.hypot(abs(self.generatedPath[startPoint + 1]["x"] - self.generatedPath[startPoint]["x"]), abs(self.generatedPath[startPoint + 1]["y"] - self.generatedPath[startPoint]["y"]))
        while startPoint < endPoint:
            generatedPoint = self.generatedPath[startPoint]
            firstX = generatedPoint["x"]
            firstY = generatedPoint["y"]
            secondX = self.generatedPath[startPoint + 1]["x"]
            secondY = self.generatedPath[startPoint + 1]["y"]
            differenceX = secondX - firstX
            differenceY = secondY - firstY
            self.pathRemainder += math.hypot(differenceX, differenceY)
            startPoint += 1
    
    def generatePath(self, points, numberOfPoints):
        self.points = points
        interpolatedList = []
        self.pointIndex = 0
        self.speedThreshold = 1
        self.targetAngle = 0
        self.startAngle = 0
        self.index = 0
        for i in range(0,numberOfPoints):
            interpolatedList.append(self.getPathPosition(i*(len(self.points)-1)/numberOfPoints))
        interpolatedList[numberOfPoints - 1]["stop"] = True
        return interpolatedList
    
    def computeRelativeLength(self, controlPoints):
        length = 0
        k = 0
        while (k < len(controlPoints) - 1):
            point = controlPoints[k]
            nextPoint = controlPoints[k + 1]
            differenceX = nextPoint.x - point.x
            differenceY = nextPoint.y - point.y
            length += math.sqrt(differenceX * differenceX + differenceY * differenceY)
            k += 1
        return int(length)
    
    def getPathPosition(self, t):
        point = {
            "x": 0,
            "y": 0,
            "speed": 0,
            "heading": 0,
            "stop": False,
            "actions": [],
            "index": self.index
        }
        self.index += 1
        if t > (len(self.points) - 1):
            return(None)
        if t > self.pointIndex:
            self.speedThreshold = self.points[self.pointIndex].speed
            point["stop"] = self.points[self.pointIndex].stop
            point["actions"] = self.points[self.pointIndex].actions
            self.startAngle = self.points[self.pointIndex].heading
            self.targetAngle = self.points[self.pointIndex + 1].heading
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
    
    def loadPath(self, pathName):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, f"paths/{pathName}.json")
        with open (filePath, "r") as f1:
            f2 = json.load(f1)
        return f2
    
    def writePath(self, path, pathName):
        ''' Only meant for code testing purposes '''
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, f"paths/{pathName}.json")
        with open (filePath, "w") as f1:
            f1.write(json.dumps(path))
    
    def convertToXY(self, xDistance, yDistance, speed):
        hypotenuse = math.hypot(xDistance, yDistance)
        ratioX = xDistance / hypotenuse
        ratioY = yDistance / hypotenuse
        x = ratioX * speed
        y = ratioY * speed
        return (x, y)
    
    def passedTargetCheck(self, currentX, currentY, nextX, nextY):
        checkOne = False
        checkTwo = False
        xDifference = nextX - currentX
        yDifference = nextY - currentY
        if xDifference < 0 and currentX < nextX:
            checkOne = True
        elif xDifference > 0 and currentX > nextX:
            checkOne = True
        if yDifference < 0 and currentY < nextY:
            checkTwo = True
        elif yDifference > 0 and currentY > nextY:
            checkTwo = True
        if checkOne and checkTwo:
            passed = True
        else:
            passed = False
        return passed, xDifference, yDifference
    
    def periodic(self, pose):
        ''' Call this function every autonomous runtime loop '''
        xPos = (pose[0] * 100) + self.xOffset
        yPos = (pose[1] * 100) + self.yOffset
        rot = (pose[2] * 100)
        print(f"xPos: {xPos}, yPos: {yPos}, rot: {rot}")
        targetPoint = self.generatedPath[self.pathPosition]
        if self.waiting == 0:
            while self.passedTargetCheck(xPos, yPos, targetPoint["x"], targetPoint["y"])[0]:
                if targetPoint["stop"]:
                    self.waiting = float(targetPoint["actions"]["waitTime"])
                    self.Timer.start()
                    return 0, 0, 0, targetPoint["actions"]
                if self.pathPosition == len(self.generatedPath):
                    return 0, 0, 0, ["end"]
                self.pathPosition += 1
                targetPoint = self.generatedPath[self.pathPosition]
            else:
                if targetPoint["stop"]:
                    self.waiting = float(targetPoint["actions"]["waitTime"])
                    self.Timer.start()
                    return 0, 0, 0, targetPoint["actions"]
                actions = targetPoint["actions"]
                passed, xDifference, yDifference = self.passedTargetCheck(xPos, yPos, targetPoint["x"], targetPoint["y"])
                takeAway = self.previousRemainder - math.hypot(abs(xDifference), abs(yDifference))
                self.previousRemainder = math.hypot(abs(xDifference), abs(yDifference))
                self.pathRemainder -= takeAway
                self.speedController.setSetpoint(-(self.OGRemainder - self.pathRemainder))
                robotSpeed = self.speedController.calculate(-self.pathRemainder)
                z = self.headingController.calculate(rot, targetPoint["heading"])
                print(f"xDifference: {xDifference}, yDifference: {yDifference}, Speed: {robotSpeed}, PathRemainder: {self.pathRemainder}")
                x, y = self.convertToXY(xDifference, yDifference, robotSpeed * targetPoint["speed"])
                return x, y, z, actions
        else:
            if self.Timer.hasElapsed(self.waiting):
                self.waiting = 0
                return 0, 0, 0, []
            else:
                return 0, 0, 0, []
            
                    
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