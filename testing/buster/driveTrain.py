from pickletools import optimize
from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import math
import ctre
import json
import os
import wpilib
import wpimath.controller
# TODO: Figure out angle conversion stuff bc it still might be in unit circle -180->180, and also find an actual talonFX brake function

class driveTrain:
    def __init__(self, config: dict):
        self.swerveModules = {
            "frontLeft": None,
            "frontRight": None,
            "rearLeft": None,
            "rearRight": None
        }
        self.config = config
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.fieldOrient = bool(self.config["RobotDefaultSettings"]["fieldOrient"])
        # LConfig = self.config["SwerveModules"]["frontLeft"]
        for i in range(4):
            moduleName = list(self.config["SwerveModules"])[i]
            swerveConfig = self.config["SwerveModules"][moduleName]
            self.swerveModules[moduleName] = swerveModule(swerveConfig["motor_ID_1"], swerveConfig["motor_ID_2"], swerveConfig["encoder_ID"], swerveConfig["encoderOffset"], moduleName)
            self.swerveModules[moduleName].initMotorEncoder()

    def rotateCartesianPlane(self, angle: float, x: float, y: float):
        newX = x*math.sin(angle) - y*math.cos(angle)
        newY = x*math.cos(angle) + y*math.sin(angle)
        return(newX, newY)

    def manualMove(self, joystickX: float, joystickY: float, joystickRotation: float, angle: float):
        '''
        This method takes the joystick inputs from the driverStation class. 
        First checking to see if it is field oriented and compensating for the navx angle if it is.
        NOTE: The final angle may be in unit circle degrees and not in normal oriented degrees this is most likely the problem if the drivetrain has a 90 degree offset
        '''
        
        #The joysticks y axis is inverted for some reason
        joystickY = -joystickY
        
        
        if self.fieldOrient:
            angle %= 360
            if angle < -180:
                angle += 360
            elif angle > 180:
                angle -= 360
            angleRadians = angle*math.pi/180
            translationVector = self.rotateCartesianPlane(angleRadians, joystickX, joystickY)
        else:
            translationVector = (joystickX, joystickY)

        fLRotationVectorAngle = (math.atan2(self.wheelBase, -1*self.trackWidth) - (math.pi/2))
        fRRotationVectorAngle = (math.atan2(self.wheelBase, self.trackWidth) - (math.pi/2))
        rLRotationVectorAngle = (math.atan2(-1*self.wheelBase, -1*self.trackWidth) - (math.pi/2))
        rRRotationVectorAngle = (math.atan2(-1*self.wheelBase, self.trackWidth) - (math.pi/2))

        fLRotationVector = (joystickRotation*math.cos(fLRotationVectorAngle), joystickRotation*math.sin(fLRotationVectorAngle))
        fRRotationVector = (joystickRotation*math.cos(fRRotationVectorAngle), joystickRotation*math.sin(fRRotationVectorAngle))
        rLRotationVector = (joystickRotation*math.cos(rLRotationVectorAngle), joystickRotation*math.sin(rLRotationVectorAngle))
        rRRotationVector = (joystickRotation*math.cos(rRRotationVectorAngle), joystickRotation*math.sin(rRRotationVectorAngle))
        
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

        self.swerveModules["frontLeft"].move(fLSpeed, fLAngle)
        self.swerveModules["frontRight"].move(fRSpeed, fRAngle)
        self.swerveModules["rearLeft"].move(rLSpeed, rLAngle)
        self.swerveModules["rearRight"].move(rRSpeed, rRAngle)

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
    
class swerveModule:
    def __init__(self, driveID: int, turnID: int, absoluteID: int, absoluteOffset: float, moduleName: str):
        if moduleName == "frontLeft":
            # do something
            kPTurn, kITurn, kDTurn = 0.005, 0.003, 0
        elif moduleName == "frontRight":
            # do something
            kPTurn, kITurn, kDTurn = 0.005, 0.003, 0
        elif moduleName == "rearLeft":
            # do something
            kPTurn, kITurn, kDTurn = 0.005, 0.003, 0
        elif moduleName == "rearRight":
            # do something
            kPTurn, kITurn, kDTurn = 0.005, 0.003, 0
            
        self.CPR = 2048
        self.turningGearRatio = 12.8 # The steering motor gear ratio
        self.drivingGearRatio = 8.14 # The driving motor gear ratio
        self.speedLimitingFactor = 0.5
        self.moduleName = moduleName
        self.absoluteOffset = -absoluteOffset
        
        self.driveMotor = ctre.TalonFX(driveID)
        self.turnMotor = ctre.TalonFX(turnID)
        
        self.absoluteEncoder = ctre.CANCoder(absoluteID)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.absoluteEncoder.configMagnetOffset(self.absoluteOffset)
        
        self.turnController = wpimath.controller.PIDController(kPTurn, kITurn, kDTurn)
        self.turnController.enableContinuousInput(-180, 180)
        self.turnController.setTolerance(0.1) # change this number to change accuracy and jitter of motor
        self.moduleReversed = False
        
        self.initMotorEncoder()
    
    def rotateUnitCircle(self, angle):
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        return(angle)
    
    def move(self, magnitude: float, angle: float):
        ''' Magnitude with an input range for 0-1, and an angle of -180->180'''
        angle = self.rotateUnitCircle(angle)
        motorPosition = self.motorPosition()
        if motorPosition > 180:
            motorPosition -= 360
        motorPosition = self.optimize(motorPosition, angle)
        if self.moduleReversed:
            magnitude = -magnitude
        self.turnController.setSetpoint(angle)
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        self.driveMotor.set(ctre.ControlMode.PercentOutput, magnitude * self.speedLimitingFactor)
        
    def stationary(self):
        ''' Keeps the swerve module still. This implementation is pretty janky tbh '''
        # may need to implement a thing for the turncontroller to still run in here if it had a previous target it never met
        self.driveMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.turnMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.brake()
        
    def coast(self):
        ''' Coasts the swerve module '''
        self.driveMotor.setsetNeutralMode(ctre.NeutralMode.Coast)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Coast)
    
    def brake(self):
        ''' Brakes the swerve module '''
        self.driveMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.turnMotor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def initMotorEncoder(self):
        ''' Called to actually set the encoder zero based off of absolute offset and position '''
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.turnMotor.setSelectedSensorPosition(int(self.absoluteEncoder.getAbsolutePosition() * self.CPR * self.turningGearRatio / 360))
        
    def motorPosition(self):
        '''motorPosition = ((self.turnMotor.getSelectedSensorPosition(0) % (self.CPR*self.turningGearRatio)) * 360/(self.CPR*self.turningGearRatio))
        if motorPosition > 180:
            motorPosition -= 360'''
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
        
        return motorPosition
    
    def enableToZero(self):
        motorPosition = self.motorPosition()
        self.turnController.setSetpoint(0) # may have to change this to 180
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        
    def optimize(self, moduleAngle, moduleTarget):
        normal = abs(moduleAngle - moduleTarget)
        oppositeAngle = moduleAngle - 180
        if oppositeAngle < -180:
            oppositeAngle += 360
        opposite = abs(oppositeAngle - moduleTarget)
        if opposite < normal:
            self.directionReversed = True
            return oppositeAngle
        else:
            self.directionReversed = False
            return moduleAngle
    
    def returnValues(self):
        motorPosition = self.motorPosition()
        rawAbsolute = self.absoluteEncoder.getAbsolutePosition() - self.absoluteOffset
        return (motorPosition, self.absoluteEncoder.getAbsolutePosition(), self.absoluteOffset, rawAbsolute) 