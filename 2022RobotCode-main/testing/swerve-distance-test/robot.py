from ctre._ctre import TalonFXPIDSetConfiguration
import wpilib
import ctre
from ctre import AbsoluteSensorRange, SensorInitializationStrategy
import math
import wpilib.controller

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        ''''''
        self.turnMotor = ctre.TalonFX(1)
        self.driveMotor = ctre.TalonFX(3)
        
        turnOffset = -20
        self.absoluteEncoder = ctre.CANCoder(2)
        self.absoluteEncoder.configMagnetOffset(turnOffset)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.turnController = wpilib.controller.PIDController(0.005, 0.0025, 0)
        self.turnController.enableContinuousInput(-180, 180)
        self.turnMotor.setSelectedSensorPosition(self.absoluteEncoder.getAbsolutePosition() * 2048 * 12.8 / 360)
        
        #self.driveMotor.configurePID(TalonFXPIDSetConfiguration.)
        wheelRadius = 2 # inches
        self.wheelCircumference = 2 * math.pi * wheelRadius # inches
        self.driveController = wpilib.controller.PIDController(0.025, 0.002, 0)
        self.driveController.setTolerance(1)
        self.driveMotor.setInverted(True)
        self.driveMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.driveMotor.setSelectedSensorPosition(0)
        self.driveMotor.setInverted(True)
        
        self.maxMPH = 0
        
    def goDistance(self, distance):
        speedFactor = 1
        wheelPosition = self.driveMotorPositionInches()
        self.driveController.setSetpoint(distance)
        speed = self.driveController.calculate(wheelPosition)
        self.driveMotor.set(ctre.ControlMode.PercentOutput, speed * speedFactor)
    
    def disabledPeriodic(self) -> None:
        self.driveMotorPositionInches()
    
    def teleopInit(self) -> None:
        '''self.distanceList = [self.wheelCircumference]
        self.madeItToTarget = False
        self.driveMotor.setse
        self.goDistance(self.wheelCircumference)'''
        self.driveMotor.setSelectedSensorPosition(0)
        
    def driveMotorPositionInches(self):
        rawPosition = self.driveMotor.getSelectedSensorPosition(0)
        positionDegrees = rawPosition * 360 / (2048 * 8.14)
        positionInches = positionDegrees * (self.wheelCircumference / 360) # maybe divide circumference by 360?
        rawVelocityPerSecond = self.driveMotor.getSelectedSensorVelocity(0) * 10
        velocityInches = (rawVelocityPerSecond * 360 / (2048 * 8.14)) * (self.wheelCircumference / 360)
        velocityMPH = (velocityInches * 60 * 60) / 63360
        if velocityMPH > self.maxMPH:
            self.maxMPH = velocityMPH
        wpilib.SmartDashboard.putNumber("Position Raw:", rawPosition)
        wpilib.SmartDashboard.putNumber("Position Degrees:", positionDegrees)
        wpilib.SmartDashboard.putNumber("Position Inches:", positionInches)
        wpilib.SmartDashboard.putNumber("Velocity MPH:", velocityMPH)
        wpilib.SmartDashboard.putNumber("Max Velocity MPH:", self.maxMPH)
        return positionInches
        
    
    def teleopPeriodic(self) -> None:
        '''if self.madeItToTarget:
            self.madeItToTarget = ""'''
        self.goDistance(self.wheelCircumference * 30)
            
        
    
    
if __name__ == "__main__":
    wpilib.run(MyRobot)
