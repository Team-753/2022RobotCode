import math
import time


class Odometry:
    def __init__(self, swerveModules, navx, config):
        self.config = config
        self.swerveModules = swerveModules
        self.navx = navx
        self.previousAngle = self.navx.getAngle()
        self.previousTime = time.time()
        
        self.displacementX = 0
        self.displacementY = 0
        
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"] * 0.0254 # Converted from inches to meters
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"] * 0.0254 # Converted from inches to meters
        
        self.fLRotationVectorAngle = (math.atan2(self.wheelBase, -1 * self.trackWidth) - (math.pi / 2))
        self.fRRotationVectorAngle = (math.atan2(self.wheelBase, self.trackWidth) - (math.pi / 2))
        self.rLRotationVectorAngle = (math.atan2(-1 * self.wheelBase, -1 * self.trackWidth) - (math.pi / 2))
        self.rRRotationVectorAngle = (math.atan2(-1 * self.wheelBase, self.trackWidth) - (math.pi / 2))
    
    def update(self):
        dt = time.time() - self.previousTime
        navxAngle = self.navxToOneEighty(self.navx.getYaw())
        dX = 0
        dY = 0
        for module in self.swerveModules:
            wheelVelocity = module.getDriveMotorVelocity() # meters per second
            wheelAngle = self.navxAngleToUnitCircle(self.fixAngleBounds(module.getTurnMotorPosition() + navxAngle)) * math.pi / 180 # radians
            wheelVector = (wheelVelocity * math.cos(wheelAngle), wheelVelocity * math.sin(wheelAngle))

            angularVelocity = self.navx.getRate() * math.pi / 180 # radians per second
            tangentialVelocity = angularVelocity  * math.hypot(self.trackWidth, self.wheelBase) / (2 * math.pi)
            if module.moduleName == "frontLeft":
                rotationAngle = (self.fLRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
            elif module.moduleName == "frontRight":
                rotationAngle = (self.fRRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
            elif module.moduleName == "rearLeft":
                rotationAngle = (self.rLRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
            elif module.moduleName == "rearRight":
                rotationAngle = (self.rRRotationVectorAngle) + self.navxAngleToUnitCircle(navxAngle)
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
        if angle > 180:
            angle += 180
            angle %= 360
            angle -= 180
        elif angle < -180:
            angle -= 180
            angle %= 360
            angle += 180
        return(angle)
    
    def navxAngleToUnitCircle(self, angle: float):
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        return(angle)

    def navxToOneEighty(self, angle: float):
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
    
    def getRobotPose(self):
        '''
        Returns:
        X displacement, 
        Y Displacement, 
        Robot Rotation (in non unit circle degrees)
        '''
        return self.displacementX, self.displacementY, self.navxToOneEighty(self.navx.getAngle())
