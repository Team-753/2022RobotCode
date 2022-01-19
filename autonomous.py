import wpilib
import wpilib.controller
import math
import os
import json

class autonomous:
    def __init__(self, pathName):
        with open(f"{os.path.dirname(os.path.abspath(__file__))}/paths/{pathName}.json", 'r') as plan:  
                self.path = json.load(plan)
        '''
        what the path will look like:
        [
        {
            "coordinates": {
            "x": 0,
            "y": 0
            },
            "rotation": 0,
            "speedFactor": 1,
            "stop": false,
            "wait": false
        },
        {
            "coordinates": {
            "x": 1,
            "y": 1
            },
            "rotation": 0,
            "speedFactor": 1,
            "stop": false,
            "wait": false
        }
        ]

        '''
        self.robotSpeedController = wpilib.controller.PIDController(0.005,0.0025,0)
        self.currentPointIndex = 0
    
    def passedTargetCheck(self, previousPoint, targetPoint, currentPosition):
        differenceX = targetPoint[0] - previousPoint[0]
        differenceY = targetPoint[1] - previousPoint[1]
        pointDistance = math.hypot(differenceX, differenceY)
        differenceX = currentPosition[0] - previousPoint[0]
        differenceY = currentPosition[1] - previousPoint[1]
        currentDistance = math.hypot(differenceX, differenceY)
        if currentDistance >= pointDistance:
            return True
        else:
            return False, currentDistance, differenceX, differenceX
    
    def convertToXY(self, xDistance, yDistance, speed):
        hypotenuse = math.hypot(xDistance, yDistance)
        ratioX = xDistance/hypotenuse
        ratioY = yDistance/hypotenuse
        x = ratioX*speed
        y = ratioY*speed
        return(x, y)

    def calculateRemainder(self, currentPointIndex, targetPointDistance):
        ''''''
        remaining = targetPointDistance
        for idx, point in enumerate(self.path):
            if idx > currentPointIndex and point["stop"] == True:
                endPointIndex = idx
        for idx, point in enumerate(self.path):
            if idx > currentPointIndex + 1 and idx <= endPointIndex:
                firstPoint = self.path[idx - 1]["coordinates"]
                secondPoint = self.path[idx]["coordinates"]
                xDifference = secondPoint[0] - firstPoint[0]
                yDifference = secondPoint[1] - firstPoint[1]
                remaining += math.hypot(xDifference, yDifference)
        return remaining
                
    
    def periodic(self, xDisplacement, yDisplacement):
        if len(self.path) == self.currentPointIndex:
            return 0, 0, 0
        self.currentPoint = self.path[self.currentPointIndex]
        self.targetPoint = self.path[self.currentPointIndex + 1] # index out of range errors incoming
        if self.currentPoint["wait"]: # add more to this later
            return 0, 0, 0
        else:
            passedTargetCheck = self.passedTargetCheck(self.currentPoint["coordinates"], self.targetPoint["coordinates"], (xDisplacement, yDisplacement))
            if passedTargetCheck[0]:
                # passed the point: cycle to the next set of points
                self.currentPointIndex += 1
                if len(self.path) == self.currentPointIndex:
                    return 0, 0, 0
                self.currentPoint = self.path[self.currentPointIndex]
                self.targetPoint = self.path[self.currentPointIndex + 1] # more index out of range errors
                passedTargetCheck = self.passedTargetCheck(self.currentPoint["coordinates"], self.targetPoint["coordinates"], (xDisplacement, yDisplacement))
                targetDistance = passedTargetCheck[1]
                targetXDistance = passedTargetCheck[2]
                targetYDistance = passedTargetCheck[3]
            else:
                # didnt pass the point; recalculate
                targetDistance = passedTargetCheck[1]
                targetXDistance = passedTargetCheck[2]
                targetYDistance = passedTargetCheck[3]
            distanceRemaining = self.calculateRemainder(self.currentPointIndex, targetDistance)
            self.robotSpeedController.setSetpoint(distanceRemaining)
            robotSpeed = self.robotSpeedController.calculate(0) * self.currentPoint["speedFactor"] # starting at 0 may be a bad implementation as it will think it has not yet accelerated to the target point yet as such will be slowed the whole time, a better implementation would be to grab the total distance currently traveled along the path and take the distance from starting to the stopping point for it to calculate the target
            x, y = self.convertToXY(targetXDistance, targetYDistance, robotSpeed)
            z = self.currentPoint["rotation"]
            auxiliary = () # add onto this later
            return x, y, z, auxiliary


class cubicBSpline():
    ''' Generates the actual coordinate map based off of the
    output from the autonomous path creator '''
    
    '''
    structure of what will be passed into this thing as a list:
    <controlPointList>[
        <ControlPoint>[
            <coordinates>(
                <int> x,
                <int> y
            ),
            <subPoints>[
                1x,
                1y,
                2x,
                2y
            ],
            <double> robotAngleTarget,
            <boolean> stop,
            <double> speedFactor,
            <boolean> wait,
            <actionList>[
                <string> command
            ]
        ]
    ]
    '''
    def __init__(self, controlPoints: list, n: int):
        '''
        :param controlPoints: The list of control points with their various attributes
        :param n: The density of points along the total path length, the higher the number, the greater the accuracy
        !Returns: The final list that you feed to the auto-algorithm
        '''
        self.controlPoints = controlPoints
        self.interpolatedList = []
        for i in range (0, n):
            point = []
            point.append(self.getPathPosition(i*(len(self.controlPoints) - 1) / n)) # adds the (x, y) tuple
            point.append([controlPoints[2], controlPoints[3], controlPoints[4], controlPoints[5], controlPoints[6]])
            self.interpolatedList.append(point)
        return self.interpolatedList 
            
    def getPathPosition(self, t):
            if t > (len(self.controlPoints) - 1):
                return None
            startingPoint = int(t) #Takes the floor of t

            t = t % 1

            P0 = self.controlPoints[startingPoint]
            P3 = self.controlPoints[startingPoint + 1]
            P1 = [P0[1][2], P0[1][3]]
            P2 = [P3[1][0], P3[1][1]]
            P0 = [self.controlPoints[startingPoint][0][0], self.controlPoints[startingPoint][0][1]]
            P3 = [self.controlPoints[startingPoint + 1][0][0], self.controlPoints[startingPoint + 1][0][1]]

            a = (1-t)**3
            b = 3*((1-t)**2)*(t)
            c = 3*(1-t)*((t)**2)
            d = (t)**3

            x = a*P0[0] + b*P1[0] + c*P2[0] + d*P3[0]
            y = a*P0[1] + b*P1[1] + c*P2[1] + d*P3[1]

            return(x, y)