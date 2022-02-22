import math
import time
import wpilib


class Odometry:
    def __init__(self, swerveModules, navx, config):
        self.config = config
        self.swerveModules = []
        self.swerveModules.append(swerveModules["frontLeft"])
        self.swerveModules.append(swerveModules["frontRight"])
        self.swerveModules.append(swerveModules["rearLeft"])
        self.swerveModules.append(swerveModules["rearRight"])
        self.navx = navx
        self.navx.reset()
        self.previousAngle = self.navx.getAngle()
        self.previousTime = time.time()
        
        self.displacementX = 0
        self.displacementY = 0
        
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"] * 0.0254 # Converted from inches to meters
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"] * 0.0254 # Converted from inches to meters
        
        self.fLRotationVectorAngle = (math.atan2(self.wheelBase, -self.trackWidth) + (math.pi / 2))
        wpilib.SmartDashboard.putNumber("fLTangent", self.fLRotationVectorAngle)
        self.fRRotationVectorAngle = (math.atan2(self.wheelBase, self.trackWidth) + (math.pi / 2))
        wpilib.SmartDashboard.putNumber("fRTangent", self.fRRotationVectorAngle)
        self.rLRotationVectorAngle = (math.atan2(-self.wheelBase, -self.trackWidth) + (math.pi / 2))
        wpilib.SmartDashboard.putNumber("rLTangent", self.rLRotationVectorAngle)
        self.rRRotationVectorAngle = (math.atan2(-self.wheelBase, self.trackWidth) + (math.pi / 2))
        wpilib.SmartDashboard.putNumber("rRTangent", self.rRRotationVectorAngle)
    
    def update(self):
        dt = time.time() - self.previousTime
        navxAngle = self.navxToOneEighty(self.navx.getYaw())
        dX = 0
        dY = 0
        for module in self.swerveModules:
            #print()
            #print(str(module.moduleName))
            wheelVelocity = module.getDriveMotorVelocity() # meters per second
            #print("navxAngle " + str(self.navxAngleToUnitCircle(navxAngle)))
            #print("moduleAngle " + str(module.getTurnMotorPosition()))
            #print("combinedAngle " + str(self.fixAngleBounds(self.navxAngleToUnitCircle(navxAngle) + module.getTurnMotorPosition())))
            wheelAngle = self.fixAngleBounds(self.navxAngleToUnitCircle(navxAngle) + module.getTurnMotorPosition()) * math.pi / 180 # radians
            wheelVector = (wheelVelocity * math.cos(wheelAngle), wheelVelocity * math.sin(wheelAngle))

            angularVelocity = self.navx.getRate() * math.pi / 180 # radians per second
            tangentialVelocity = angularVelocity  * math.hypot(self.trackWidth, self.wheelBase) / (2 * math.pi)
            if module.moduleName == "frontLeft":
                rotationAngle = (self.fLRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
                wpilib.SmartDashboard.putNumber("fLWheelVectorX", wheelVector[0])
                wpilib.SmartDashboard.putNumber("fLWheelVectorY", wheelVector[1])
                wpilib.SmartDashboard.putNumber("fLWheelVelocity", wheelVelocity)
                wpilib.SmartDashboard.putNumber("fLWheelAngle", wheelAngle)
            elif module.moduleName == "frontRight":
                rotationAngle = (self.fRRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
                wpilib.SmartDashboard.putNumber("fRWheelVectorX", wheelVector[0])
                wpilib.SmartDashboard.putNumber("fRWheelVectorY", wheelVector[1])
                wpilib.SmartDashboard.putNumber("fRWheelVelocity", wheelVelocity)
                wpilib.SmartDashboard.putNumber("fRWheelAngle", wheelAngle)
            elif module.moduleName == "rearLeft":
                rotationAngle = (self.rLRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
                wpilib.SmartDashboard.putNumber("rLWheelVectorX", wheelVector[0])
                wpilib.SmartDashboard.putNumber("rLWheelVectorY", wheelVector[1])
                wpilib.SmartDashboard.putNumber("rLWheelVelocity", wheelVelocity)
                wpilib.SmartDashboard.putNumber("rLWheelAngle", wheelAngle)
            elif module.moduleName == "rearRight":
                rotationAngle = (self.rRRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
                wpilib.SmartDashboard.putNumber("rRWheelVectorX", wheelVector[0])
                wpilib.SmartDashboard.putNumber("rRWheelVectorY", wheelVector[1])
                wpilib.SmartDashboard.putNumber("rRWheelVelocity", wheelVelocity)
                wpilib.SmartDashboard.putNumber("rRWheelAngle", wheelAngle)
            rotationVector = (tangentialVelocity * math.cos(rotationAngle), tangentialVelocity * math.sin(rotationAngle))

            xVelocity = wheelVector[0] - rotationVector[0]
            yVelocity = wheelVector[1] - rotationVector[1]

            dX += xVelocity * dt
            dY += yVelocity * dt
        
        dX /= 4
        dY /= 4

        self.displacementX += dX
        self.displacementY += dY
    
        self.previousAngle = self.navx.getAngle()
        self.previousTime = time.time()
    
    def fixAngleBounds(self, angle: float):
        angle += 180
        angle %= 360
        angle -= 180
        return(angle)
    
    def navxAngleToUnitCircle(self, angle: float):
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        angle = -angle
        return(angle)

    def navxToOneEighty(self, angle: float):
        angle %= 360
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        '''if angle < -90:
            angle += 270
        else:
            angle -= 90'''
        return(angle)
    
    def getRobotPose(self):
        '''
        Returns:
        X displacement, 
        Y Displacement, 
        Robot Rotation (in non unit circle degrees)
        '''
        return self.displacementX, self.displacementY, self.navxAngleToUnitCircle(self.navxToOneEighty(self.navx.getAngle()))
    
    def reset(self):
        self.displacementX = 0
        self.displacementY = 0
