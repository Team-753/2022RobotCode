import wpilib

import math
import navx
import ctre
import json
import os

class driveTrain:
    def __init__(self, config):
        self.config = config
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.fieldOrient = self.config["RobotDefaultSettings"]["fieldOrient"]
        
        fLConfig = self.config["SwerveModules"]["frontLeft"]
        fRConfig = self.config["SwerveModules"]["frontRight"]
        rLConfig = self.config["SwerveModules"]["rearLeft"]
        rRConfig = self.config["SwerveModules"]["rearRight"]
        
        self.frontLeft = swerveModule(fLConfig["motor_ID_1"], fLConfig["motor_ID_2"], fLConfig["encoderID"], fLConfig["encoderOffset"], "frontLeft")
        self.frontRight = swerveModule(fRConfig["motor_ID_1"], fRConfig["motor_ID_2"], fRConfig["encoderID"], fRConfig["encoderOffset"], "frontRight")
        self.rearLeft = swerveModule(rLConfig["motor_ID_1"], rLConfig["motor_ID_2"], rLConfig["encoderID"], rLConfig["encoderOffset"], "rearLeft")
        self.rearRight = swerveModule(rRConfig["motor_ID_1"], rRConfig["motor_ID_2"], rRConfig["encoderID"], rRConfig["encoderOffset"], "rearRight")
        self.frontLeft.initMotorEncoder()
        self.frontRight.initMotorEncoder()
        self.rearLeft.initMotorEncoder()
        self.rearRight.initMotorEncoder()

    def rotateCartesianPlane(self, angle, x, y):
        newX = x*math.sin(angle) - y*math.cos(angle)
        newY = x*math.cos(angle) + y*math.sin(angle)
        return(newX, newY)

    def manualControl(self, joystickX, joystickY, joystickRotation):
        '''
        This method takes the joystick inputs from the driverStation class. 
        First checking to see if it is field oriented and compensating for the navx angle if it is.
        
        '''
        if self.fieldOrient:
            angle = -1*navx.getAngle() + 90
            angle %= 360
            if angle < -180:
                angle += 360
            elif angle > 180:
                angle -= 360
            angleRadians = angle*math.pi/180
            translationVector = self.rotateCartesianPlane(angleRadians, joystickX, joystickY)
        else:
            translationVector = (joystickX, joystickY)

        fLRotationVectorAngle = joystickRotation*(math.atan2(self.wheelBase, -1*self.trackWidth) - (math.pi/2))
        fRRotationVectorAngle = joystickRotation*(math.atan2(self.wheelBase, self.trackWidth) - (math.pi/2))
        rLRotationVectorAngle = joystickRotation*(math.atan2(-1*self.wheelBase, -1*self.trackWidth) - (math.pi/2))
        rRRotationVectorAngle = joystickRotation*(math.atan2(-1*self.wheelBase, self.trackWidth) - (math.pi/2))

        fLRotationVector = (math.cos(fLRotationVectorAngle), math.sin(fLRotationVectorAngle))
        fRRotationVector = (math.cos(fRRotationVectorAngle), math.sin(fRRotationVectorAngle))
        rLRotationVector = (math.cos(rLRotationVectorAngle), math.sin(rLRotationVectorAngle))
        rRRotationVector = (math.cos(rRRotationVectorAngle), math.sin(rRRotationVectorAngle))

        fLTranslationVector = (fLRotationVector[0] + translationVector[0], fLRotationVector[1] + translationVector[1])
        fRTranslationVector = (fRRotationVector[0] + translationVector[0], fRRotationVector[1] + translationVector[1])
        rLTranslationVector = (rLRotationVector[0] + translationVector[0], rLRotationVector[1] + translationVector[1])
        rRTranslationVector = (rRRotationVector[0] + translationVector[0], rRRotationVector[1] + translationVector[1])

        fLAngle = math.atan2(fLTranslationVector[1], fLTranslationVector[0])*180/math.pi
        fRAngle = math.atan2(fRTranslationVector[1], fRTranslationVector[0])*180/math.pi
        rLAngle = math.atan2(rLTranslationVector[1], rLTranslationVector[0])*180/math.pi
        rRAngle = math.atan2(rRTranslationVector[1], rRTranslationVector[0])*180/math.pi

        fLSpeed = math.sqrt((fLTranslationVector[0]**2) + (fLTranslationVector[1]**2))
        fRSpeed = math.sqrt((fRTranslationVector[0]**2) + (fRTranslationVector[1]**2))
        rLSpeed = math.sqrt((rLTranslationVector[0]**2) + (rLTranslationVector[1]**2))
        rRSpeed = math.sqrt((rRTranslationVector[0]**2) + (rRTranslationVector[1]**2))
    
        maxSpeed = max(fLSpeed, fRSpeed, rLSpeed, rRSpeed)
        if maxSpeed > 1:
            fLSpeed /= maxSpeed
            fRSpeed /= maxSpeed
            rLSpeed /= maxSpeed
            rRSpeed /= maxSpeed

        self.frontLeft.move(fLSpeed, fLAngle)
        self.frontRight.move(fRSpeed, fRAngle)
        self.rearLeft.move(rLSpeed, rLAngle)
        self.rearRight.move(rRSpeed, rRAngle)
    
    def zeroMotorEncoders(self):
        ''' Call this when actually re-zeroing the motor absolutes '''
        self.frontLeft.zeroMotorEncoder()
        self.frontRight.zeroMotorEncoder()
        self.reaerLeft.zeroMotorEncoder()
        self.rearRight.zeroMotorEncoder()
        
    def stationary(self):
        ''' Makes the robot's drivetrain stationary '''
        self.frontLeft.stationary()
        self.frontRight.stationary()
        self.rearLeft.stationary()
        self.rearRight.stationary()

    def coast(self):
        ''' Coasts the robot's drivetrain '''
        self.frontLeft.coast()
        self.frontRight.coast()
        self.rearLeft.coast()
        self.rearRight.coast()
        
    def refreshValues(self):
        frontLeftValues = self.frontLeft.returnValues()
        frontRightValues = self.frontRight.returnValues()
        rearLeftValues = self.rearLeft.returnValues()
        rearRightValues = self.rearRight.returnValues()
        wpilib.SmartDashboard.putNumberArray("frontLeft", frontLeftValues)
        wpilib.SmartDashboard.putNumberArray("frontRight", frontRightValues)
        wpilib.SmartDashboard.putNumberArray("rearLeft", rearLeftValues)
        wpilib.SmartDashboard.putNumberArray("rearRight", rearRightValues)
    
class swerveModule:
    def __init__(self, driveID, turnID, absoluteID, absoluteOffset, moduleName):
        if moduleName == "frontLeft":
            # do something
            kPTurn, kITurn, kDTurn = 0, 0, 0
        elif moduleName == "frontRight":
            # do something
            kPTurn, kITurn, kDTurn = 0, 0, 0
        elif moduleName == "rearLeft":
            # do something
            kPTurn, kITurn, kDTurn = 0, 0, 0
        elif moduleName == "rearRight":
            # do something
            kPTurn, kITurn, kDTurn = 0, 0, 0
        self.CPRConversionFactor = 2048 / 360
        self.turningGearRatio = 12.8
        self.moduleName = moduleName
        
        self.driveMotor = ctre.TalonFX(driveID)
        self.driveMotor.configFactoryDefault()
        self.driveMotorControlMode = ctre.TalonFXControlMode.PercentOutput

        self.turnMotor = ctre.TalonFX(turnID)
        self.turnMotor.configFactoryDefault()
        self.turnMotor.configIntegratedSensorAbsoluteRange((-180, 180))
        self.turnMotor.configurePID((kPTurn, kITurn, kDTurn))
        self.turnMotorControlMode = ctre.TalonFXControlMode.Position
        
        self.absoluteEncoder = ctre.CANCoder(absoluteID)
        self.absoluteEncoder.configAbsoluteSensorRange((-180, 180))
        self.absoluteOffset = absoluteOffset
        self.initMotorEncoder()
    
    def move(self, magnitude, angle):
        ''' Magnitude with an input range for 0-1, and an angle of 0-360'''
        encoderTarget = angle * self.CPRConversionFactor * self.turningGearRatio
        self.turnMotor.set(self.turnMotorControlMode, encoderTarget)
        self.driveMotor.set(self.driveMotorControlMode, magnitude)
        
    def stationary(self):
        ''' Brakes and keeps the swerve module still '''
        self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, 0)
        
    def coast(self):
        ''' Coasts the swerve module '''
        self.driveMotor.set(ctre.ControlMode.Disabled, 0)
        self.turnMotor.set(ctre.ControlMode.Disabled, 0)
    
    def initMotorEncoder(self):
        ''' Called to actually set the encoder zero based off of absolute offset and position '''
        position = self.absoluteEncoder.getAbsolutePosition() - self.absoluteOffset
        self.turnMotor.setSelectedSensorPosition(position, 0)
        self.turnMotor.set(self.turnMotorControlMode, 0)
        
    def zeroMotorEncoder(self):
        ''' Call this when physically setting the motor encoder zeros '''
        self.absoluteOffset = self.absoluteEncoder.getAbsolutePosition() # write that to the json
        self.rewriteZeros()
        self.initMotorEncoder()
        
    def rewriteZeros(self):
        filePath = f"{os.getcwd()}./self.config.json"
        with open(filePath, 'w') as jsonFile:
            self.config = json.load(jsonFile)
            self.config["SwerveModules"][self.moduleName]["encoderOffset"] = self.absoluteOffset
            jsonFile.write(json.dump(self.config, indent=2))
    
    def returnValues(self):
        return (self.turnMotor.getSelectedSensorPosition(0), self.absoluteEncoder.getAbsolutePosition(), self.absoluteOffset)
        