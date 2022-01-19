import math                   




class path:
    class controlPoint:
        def __init__(self, x, y, theta, d):
            self.x = x
            self.y = y
            self.theta = theta
            self.d = d

            self.subPoint1X = -1*self.d*math.cos(self.theta) + self.x
            self.subPoint1Y = -1*self.d*math.sin(self.theta) + self.y
            self.subPoint2X = self.d*math.cos(self.theta) + self.x
            self.subPoint2Y = self.d*math.sin(self.theta) + self.y

        def updateSubPoints(self):
            self.subPoint1X = -1*self.d*math.cos(self.theta) + self.x
            self.subPoint1Y = -1*self.d*math.sin(self.theta) + self.y
            self.subPoint2X = self.d*math.cos(self.theta) + self.x
            self.subPoint2Y = self.d*math.sin(self.theta) + self.y

        def setX(self, x):
            self.x = x

        def setY(self, y):
            self.y = y

        def setCoords(self, x, y):
            self.x = x
            self.y = y

        def setAngle(self, theta):
            self.theta = theta
            self.updateSubPoints()

        def setDistanceValue(self, distance):
            self.d = distance
            self.updateSubPoints()


    class cubicBSpline:
        def __init__(self, points):
            self.points = points

        def interpolatePath(self, numberOfPoints):
            self.interpolatedList = []
            for i in range(0,numberOfPoints):
                self.interpolatedList.append(self.getPathPosition(i*(len(self.points)-1)/numberOfPoints))
            return(self.interpolatedList)

        def getPathPosition(self, t):
            if t > (len(self.points) - 1):
                return(None)
            startingPoint = int(t) #Takes the floor of t

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

            return(x, y)

robotPath1 = [path.controlPoint(-200,-200,math.pi/2,0), path.controlPoint(0,0,math.pi/2,0), path.controlPoint(200, 200, math.pi/2, 55), path.controlPoint(300,300,0,55)]
#circleThing = autonomous("yolo", "editor")
# circleThing.createCirclePath((0,0), 50, 270, 180, True)