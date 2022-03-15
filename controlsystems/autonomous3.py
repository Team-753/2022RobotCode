import json
import math
import os
import wpimath.controller

class Autonomous:
    def __init__(self, planName, navx) -> None:
        self.navx = navx
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, f"paths/{planName}.json")
        with open (filePath, "r") as f1:
            unparsedPath = json.load(f1)
        self.controlPoints = []
        for controlPoint in unparsedPath:
            self.controlPoints.append(ControlPoint(controlPoint["x"], controlPoint["y"], controlPoint["theta"], controlPoint["d"], controlPoint["speed"], controlPoint["heading"], controlPoint["stop"], controlPoint["actions"]))
        self.navx.setAngleAdjustment(self.controlPoints[0].heading * 180 / math.pi)
        self.pathIndex = 1
        self.atPoint = False
        self.zPID = wpimath.controller.PIDController(0.0005, 0.0, 0)
        self.zPID.enableContinuousInput(-math.pi, math.pi)
        
    def convertToXY(self, xDistance, yDistance, speed):
        hypotenuse = math.hypot(xDistance, yDistance)
        ratioX = xDistance / hypotenuse
        ratioY = yDistance / hypotenuse
        x = ratioX * speed
        y = ratioY * speed
        return (x, y)
    
    def periodic(self, robotPose, trajectory, check):
        xDisplacement = (robotPose[0] * 100) + self.controlPoints[0].x
        yDisplacement = (robotPose[1] * 100) + self.controlPoints[0].y
        targetPoint = self.controlPoints[self.pathIndex]
        robotRotationRadians = self.navx.getAngle() * math.pi / 180
        if not targetPoint.stop and not self.atPoint:
            z = self.zPID.calculate(targetPoint.heading, robotRotationRadians)
            passed = False
            if trajectory < math.pi / 2 and trajectory > 0:
                if xDisplacement > self.controlPoints[self.pathIndex].x and yDisplacement > self.controlPoints[self.pathIndex].y:
                    passed = True
            elif trajectory < math.pi and trajectory > math.pi / 2:
                if xDisplacement < self.controlPoints[self.pathIndex].x and yDisplacement > self.controlPoints[self.pathIndex].y:
                    passed = True
            elif trajectory < -math.pi / 2 and trajectory > -math.pi:
                if xDisplacement < self.controlPoints[self.pathIndex].x and yDisplacement < self.controlPoints[self.pathIndex].y:
                    passed = True
            else:
                if xDisplacement > self.controlPoints[self.pathIndex].x and yDisplacement < self.controlPoints[self.pathIndex].y:
                    passed = True
            if passed:
                if targetPoint.stop:
                    self.atPoint = True
                    return 0, 0, 0, targetPoint.actions
            else:
                xDiff = targetPoint.x - xDisplacement
                yDiff = targetPoint.y - yDisplacement
                x, y = self.convertToXY(xDiff, yDiff, 1)
                return x, y, z, []
        elif targetPoint.stop and self.atPoint:
            pass # waiting for a check?
                    


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