from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import math
import ctre
import json
import os
import wpilib.controller

# TODO: Figure out angle conversion stuff bc it still might be in unit circle -180->180, and also find an actual talonFX brake function

class driveTrain:
    def __init__(self, config: dict):
        self.config = config
        self.trackWidth = self.config["RobotDimensions"]["trackWidth"]
        self.wheelBase = self.config["RobotDimensions"]["wheelBase"]
        self.fieldOrient = bool(self.config["RobotDefaultSettings"]["fieldOrient"])
        
        fLConfig = self.config["SwerveModules"]["frontLeft"]
        fRConfig = self.config["SwerveModules"]["frontRight"]
        rLConfig = self.config["SwerveModules"]["rearLeft"]
        rRConfig = self.config["SwerveModules"]["rearRight"]
        
        self.frontLeft = swerveModule(fLConfig["motor_ID_1"], fLConfig["motor_ID_2"], fLConfig["encoder_ID"], fLConfig["encoderOffset"], "frontLeft")
        self.frontRight = swerveModule(fRConfig["motor_ID_1"], fRConfig["motor_ID_2"], fRConfig["encoder_ID"], fRConfig["encoderOffset"], "frontRight")
        self.rearLeft = swerveModule(rLConfig["motor_ID_1"], rLConfig["motor_ID_2"], rLConfig["encoder_ID"], rLConfig["encoderOffset"], "rearLeft")
        self.rearRight = swerveModule(rRConfig["motor_ID_1"], rRConfig["motor_ID_2"], rRConfig["encoder_ID"], rRConfig["encoderOffset"], "rearRight")
        self.frontLeft.initMotorEncoder()
        self.frontRight.initMotorEncoder()
        self.rearLeft.initMotorEncoder()
        self.rearRight.initMotorEncoder()
        
        
        self.easterEgg = self.config["RobotDefaultSettings"]["easterEgg"]
        self.orchestra = ctre.Orchestra()
        self.orchestra.addInstrument(self.frontLeft.driveMotor, self.frontLeft.turnMotor, self.frontRight.driveMotor, self.frontRight.turnMotor, self.rearLeft.driveMotor, self.rearLeft.turnMotor, self.rearRight.driveMotor, self.rearRight.turnMotor)

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

        self.frontLeft.move(fLSpeed, fLAngle)
        self.frontRight.move(fRSpeed, fRAngle)
        self.rearLeft.move(rLSpeed, rLAngle)
        self.rearRight.move(rRSpeed, rRAngle)

    def reInitiateMotorEncoders(self):
        ''' Call this when actually re-zeroing the motor absolutes '''
        self.frontLeft.initMotorEncoder()
        self.frontRight.initMotorEncoder()
        self.rearLeft.initMotorEncoder()
        self.rearRight.initMotorEncoder()
        
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
        '''wpilib.SmartDashboard.putNumberArray("frontLeft", frontLeftValues)
        wpilib.SmartDashboard.putNumberArray("frontRight", frontRightValues)
        wpilib.SmartDashboard.putNumberArray("rearLeft", rearLeftValues)
        wpilib.SmartDashboard.putNumberArray("rearRight", rearRightValues)'''
        return frontLeftValues, frontRightValues, rearLeftValues, rearRightValues
        
    def easterEgg(self):
        ''' Plays a pre-set tune on the motors;
            This can be interrupted by using pause or set on the TalonFX controllers'''
        '''self.orchestra.loadMusic(f"{os.getcwd()}./tunes/{self.easterEgg}.chrp")
        self.orchestra.play()'''
        
    def enableToZeros(self):
        self.frontLeft.enableToZero()
        self.frontRight.enableToZero()
        self.rearLeft.enableToZero()
        self.rearRight.enableToZero()
    
class swerveModule:
    def __init__(self, driveID: int, turnID: int, absoluteID: int, absoluteOffset: float, moduleName: str):
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
            
        self.CPR = 2048
        self.turningGearRatio = 12.8 # The steering motor gear ratio
        self.drivingGearRatio = 8.14 # The driving motor gear ratio
        self.moduleName = moduleName
        self.absoluteOffset = absoluteOffset
        
        self.driveMotor = ctre.TalonFX(driveID)
        self.turnMotor = ctre.TalonFX(turnID)
        
        self.absoluteEncoder = ctre.CANCoder(absoluteID)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.absoluteEncoder.configMagnetOffset(self.absoluteOffset)
        
        self.turnController = wpilib.controller.PIDController(0.005, 0.0025, 0)
        self.turnController.enableContinuousInput(-180, 180)
        self.previousTarget = 0
        self.turnController.setTolerance(0.25) # change this number to change accuracy and jitter of motor
        
        self.initMotorEncoder()
    
    def move(self, magnitude: float, angle: float):
        ''' Magnitude with an input range for 0-1, and an angle of -180->180'''
        self.previousTarget = angle
        motorPosition = self.motorPosition()
        if motorPosition > 180:
            motorPosition -= 360
        self.turnController.setSetpoint(angle)
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        self.driveMotor.set(ctre.ControlMode.PercentOutput, magnitude)
        
    def stationary(self):
        ''' Keeps the swerve module still. This implementation is pretty janky tbh '''
        # may need to implement a thing for the turncontroller to still run in here if it had a previous target it never met
        self.driveMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.turnMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        
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
        self.turnMotor.setSelectedSensorPosition(self.absoluteEncoder.getAbsolutePosition() * self.CPR * self.turningGearRatio / 360)
        
    def motorPosition(self):
        motorPosition = ((self.turnMotor.getSelectedSensorPosition(0) % (self.CPR*self.turningGearRatio)) * 360/(self.CPR*self.turningGearRatio))
        if motorPosition > 180:
            motorPosition -= 360
        return motorPosition
    
    def enableToZero(self):
        motorPosition = self.motorPosition()
        self.turnController.setSetpoint(0) # may have to change this to 180
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        
    def rewriteZeros(self):
        rawAbsolute = self.absoluteEncoder.getAbsolutePosition() - self.absoluteOffset
        with open (f"{os.path.dirname(os.path.abspath(__file__))}/config.json", "r") as f1:
            config = json.load(f1)
        config["SwerveModules"][self.moduleName]["encoderOffset"] = -rawAbsolute
        with open (f"{os.path.dirname(os.path.abspath(__file__))}/config.json", "w") as f2:
            f2.write(json.dump(config, indent=2))
        self.absoluteOffset = -rawAbsolute
        self.initMotorEncoder()
    
    def returnValues(self):
        motorPosition = self.motorPosition()
        rawAbsolute = self.absoluteEncoder.getAbsolutePosition() - self.absoluteOffset
        return (motorPosition, self.absoluteEncoder.getAbsolutePosition(), self.absoluteOffset, rawAbsolute) 
