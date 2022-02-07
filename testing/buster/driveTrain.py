import math
import json
import os
import ctre
import wpilib
import wpimath.controller
import wpimath.kinematics
import wpimath.geometry
import navx # temp
from swerveModule import swerveModule

class driveTrain:
    def __init__(self, config: dict, navxOBJ):
        self.navx = navx.AHRS() # self.navx = navxOBJ
        self.swerveModules = {
            "frontLeft": None,
            "frontRight": None,
            "rearLeft": None,
            "rearRight": None
        }
        
        inchesToMeters = 0.0254
        frontLeftLocation = wpimath.geometry.Translation2d(self.config["RobotDimensions"]["trackWidth"] * inchesToMeters / 2, self.config["RobotDimensions"]["wheelBase"] * inchesToMeters / 2)
        frontRightLocation = wpimath.geometry.Translation2d(self.config["RobotDimensions"]["trackWidth"] * inchesToMeters / 2, -1*self.config["RobotDimensions"]["wheelBase"] * inchesToMeters / 2)
        rearLeftLocation = wpimath.geometry.Translation2d(-1*self.config["RobotDimensions"]["trackWidth"] * inchesToMeters / 2, self.config["RobotDimensions"]["wheelBase"] * inchesToMeters / 2)
        rearRightLocation = wpimath.geometry.Translation2d(-1*self.config["RobotDimensions"]["trackWidth"] * inchesToMeters / 2, -1*self.config["RobotDimensions"]["wheelBase"] * inchesToMeters / 2)
  
        self.config = config
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.fieldOrient = bool(self.config["RobotDefaultSettings"]["fieldOrient"])
        self.maxSpeed = self.config["RobotConstraints"]["MaxSpeed"]
        # LConfig = self.config["SwerveModules"]["frontLeft"]
        for i in range(4):
            moduleName = list(self.config["SwerveModules"])[i]
            swerveConfig = self.config["SwerveModules"][moduleName]
            self.swerveModules[moduleName] = swerveModule(swerveConfig["motor_ID_1"], swerveConfig["motor_ID_2"], swerveConfig["encoder_ID"], swerveConfig["encoderOffset"], moduleName)
            
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(frontLeftLocation, frontRightLocation, rearLeftLocation, rearRightLocation)
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(self.kinematics, self.poseRadians())

    def poseRadians(self):
        return wpimath.geometry.Rotation2d((-1*self.navx.getAngle()+90)*(math.pi / 180))

    def rotateCartesianPlane(self, angle: float, x: float, y: float):
        newX = x*math.sin(angle) - y*math.cos(angle)
        newY = x*math.cos(angle) + y*math.sin(angle)
        return(newX, newY)

    def move(self, joystickX: float, joystickY: float, joystickRotation: float, angle: float):
        ''' Sorry Ben but unless you want to do full custom inverse odometry this is what we are stuck with '''
        if self.fieldOrient:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(joystickX, joystickY, joystickRotation, self.getPoseRadians()))
        else:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(wpimath.kinematics.ChassisSpeeds(joystickX, joystickY, joystickRotation))
            
        swerveModuleStates = wpimath.kinematics.SwerveDrive4Kinematics.normalizeWheelSpeeds(swerveModuleStates, self.maxSpeed)
        self.swerveModules["frontLeft"].setState(swerveModuleStates[0])
        self.swerveModules["frontRight"].setState(swerveModuleStates[1])
        self.swerveModules["rearLeft"].setState(swerveModuleStates[2])
        self.swerveModules["rearRight"].setState(swerveModuleStates[3])

    def reInitiateMotorEncoders(self):
        ''' Call this when actually re-zeroing the motor absolutes '''
        self.swerveModules["frontLeft"].initMotorEncoder()
        self.swerveModules["frontRight"].initMotorEncoder()
        self.swerveModules["rearLeft"].initMotorEncoder()
        self.swerveModules["rearRight"].initMotorEncoder()
        
    def stationary(self):
        ''' Makes the robot's drivetrain stationary '''
        self.swerveModules["frontLeft"].stationary()
        self.swerveModules["frontRight"].stationary()
        self.swerveModules["rearLeft"].stationary()
        self.swerveModules["rearRight"].stationary()

    def coast(self):
        ''' Coasts the robot's drivetrain '''
        self.swerveModules["frontLeft"].coast()
        self.swerveModules["frontRight"].coast()
        self.swerveModules["rearLeft"].coast()
        self.swerveModules["rearRight"].coast()
        
    def refreshValues(self):
        frontLeftValues = self.swerveModules["frontLeft"].returnValues()
        frontRightValues = self.swerveModules["frontRight"].returnValues()
        rearLeftValues = self.swerveModules["rearLeft"].returnValues()
        rearRightValues = self.swerveModules["rearRight"].returnValues()
        return frontLeftValues, frontRightValues, rearLeftValues, rearRightValues
    
    def zeroAbsolutes(self):
        ''' Sets the zero of the absolute canCoders - 
        realistically this function should never have to be called'''
        self.swerveModules["frontLeft"].zeroAbsolute()
        self.swerveModules["frontRight"].zeroAbsolute()
        self.swerveModules["rearLeft"].zeroAbsolute()
        self.swerveModules["rearRight"].zeroAbsolute()
        
    def turnOnly(self, angle):
        self.swerveModules["frontLeft"].move(0, angle)
        self.swerveModules["frontRight"].move(0, angle)
        self.swerveModules["rearLeft"].move(0, angle)
        self.swerveModules["rearRight"].move(0, angle)
        
    def updateOdometry(self):
        self.odometry.update(self.getPoseRadians(), self.frontLeft.getState(), self.frontRight.getState(), self.rearLeft.getState(), self.rearRight.getState())
        
    def rotateUnitCircle(self, angle: float):
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        return(angle)
    
    def getFieldPosition(self):
        pose = self.odometry.getPose()
        return pose.X(), pose.Y(), self.rotateUnitCircle(math.degrees(pose.rotation())) # rotating may be un necessary