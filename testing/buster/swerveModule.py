import ctre
import wpimath.kinematics
import math
import os
import json
import wpimath.controller
import wpimath.trajectory

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
        
        kPDrive = 1
        kIDrive = 0
        kDDrive = 0
        kFDrive = .108
            
        maxAngularVelocity = math.pi # Radians per second
        maxAngularAcceleration = 2*math.pi # Radians per second squared
        self.CPR = 2048
        self.turningGearRatio = 12.8 # The steering motor gear ratio
        self.drivingGearRatio = 8.14 # The driving motor gear ratio
        self.speedLimitingFactor = 0.5 # possibly temporary
        self.wheelDiameter = .1016 # Meters
        self.moduleName = moduleName
        self.absoluteOffset = -absoluteOffset
        
        self.driveMotor = ctre.TalonFX(driveID)
        self.turnMotor = ctre.TalonFX(turnID)
        
        self.absoluteEncoder = ctre.CANCoder(absoluteID)
        self.absoluteEncoder.configSensorInitializationStrategy(ctre.SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(ctre.AbsoluteSensorRange.Signed_PlusMinus180)
        self.absoluteEncoder.configMagnetOffset(self.absoluteOffset)
        
        self.driveMotor.config_kP(0, kPDrive, 0)
        self.driveMotor.config_kI(0, kIDrive, 0)
        self.driveMotor.config_kD(0, kDDrive, 0)
        self.driveMotor.config_kF(0, kFDrive, 0)
        
        self.turnController = wpimath.controller.ProfiledPIDControllerRadians(kPTurn, kITurn, kDTurn, 
        wpimath.trajectory.TrapezoidProfileRadians.Constraints(maxAngularVelocity, maxAngularAcceleration))
        self.turnController.enableContinuousInput(-180, 180)
        self.turnController.setTolerance(0.005) # change this number to change accuracy and jitter of motor
        self.turnController.enableContinuousInput(-math.pi, math.pi)
        # self.moduleReversed = False depracated
        
        self.initMotorEncoder()
    
    def rotateUnitCircle(self, angle: float):
        if angle < -90:
            angle += 270
        else:
            angle -= 90
        return(angle)
    
    '''def move(self, magnitude: float, angle: float):
        # Magnitude with an input range for 0-1, and an angle of -180->180
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
        self.driveMotor.set(ctre.ControlMode.PercentOutput, magnitude * self.speedLimitingFactor)'''
        
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
        ''' Called to set the encoder zero based off of absolute offset and position '''
        self.turnMotor.configIntegratedSensorAbsoluteRange(ctre.AbsoluteSensorRange.Unsigned_0_to_360)
        self.turnMotor.setSelectedSensorPosition(int(self.absoluteEncoder.getAbsolutePosition() * self.CPR * self.turningGearRatio / 360))
        self.driveMotor.setSelectedSensorPosition(0)
        
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
        
    '''def optimize(self, moduleAngle: float, moduleTarget: float):
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
    '''
        
    def zeroAbsolute(self):
        ''' Realistically this function should NEVER have to be called but
        just in case... '''
        currentActualPosition = self.absoluteEncoder.getAbsolutePosition() + self.absoluteOffset # maybe negative?
        self.absoluteEncoder.configMagnetOffset(currentActualPosition)
        self.initMotorEncoder()
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            config = json.load(f1)
        config["SwerveModules"][self.moduleName]["encoderOffset"] = currentActualPosition
        with open (filePath, "w") as f2:
            f2.write(json.dumps(config))
    
    def returnValues(self):
        motorPosition = self.motorPosition()
        rawAbsolute = self.absoluteEncoder.getAbsolutePosition() - self.absoluteOffset
        return (motorPosition, self.absoluteEncoder.getAbsolutePosition(), self.absoluteOffset, rawAbsolute) 
    
    def wheelPosition(self):
        ''' Returns wheel position in radians from -pi to pi.'''
        wheelPositionRadians = ((self.turnMotor.getSelectedSensorPosition(0) % (self.CPR*self.turningGearRatio)) * 2 * math.pi/(self.CPR*self.turningGearRatio))
        if wheelPositionRadians > math.pi:
            wheelPositionRadians -= 2*math.pi
        return wheelPositionRadians 
    
    def getState(self):
        speedMetersPerSecond = self.driveMotor.getSelectedSensorVelocity(0) * 10 * self.wheelDiameter * math.pi / self.CPR # Talon works with velocity in ticks/100ms
        return wpimath.kinematics.SwerveModuleState(speedMetersPerSecond, wpimath.geometry.Rotation2d(self.wheelPosition())) 

    def setState(self, desiredState):
        state = wpimath.kinematics.SwerveModuleState.optimize(desiredState, wpimath.geometry.Rotation2d(self.wheelPosition())) # Update with actual encoder code
        velocity = state.speed * self.CPR / (10 * self.wheelDiameter * math.pi) # Converting from m/s to ticks/100ms
        turnOutput = self.turnController.calculate(self.wheelPosition(), state.angle.radians())
        self.driveMotor.set(ctre.TalonFXControlMode.Velocity, velocity)
		#I'm not sure how to scale these units so I'll leave (careful) testing of that to you. Or you could use the direct position PID
        self.turnMotor.set(ctre.TalonFXControlMode.Current, turnOutput)