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
        self.driveController = wpilib.controller.PIDController(0.005, 0.0025, 0)
        self.driveMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.driveMotor.setSelectedSensorPosition(0)
        
    def goDistance(self, distance):
        #self.driveMotor.set(ctre.ControlMode.MotionMagic, distance)
        pass
    
    def disabledPeriodic(self) -> None:
        self.driveMotorPositionInches()
    
    def teleopInit(self) -> None:
        '''self.distanceList = [self.wheelCircumference]
        self.madeItToTarget = False
        self.driveMotor.setse
        self.goDistance(self.wheelCircumference)'''
        
    def driveMotorPositionInches(self):
        rawPosition = self.driveMotor.getSelectedSensorPosition(0)
        positionDegrees = (rawPosition % (2048 * 8.14)) * 360 / (2048 * 8.14)
        positionInches = positionDegrees * self.wheelCircumference
        wpilib.SmartDashboard.putNumber("Position Degrees:", positionDegrees)
        wpilib.SmartDashboard.putNumber("Position Inches:", positionInches)
        
    
    def teleopPeriodic(self) -> None:
        '''if self.madeItToTarget:
            self.madeItToTarget = ""'''
        
    
    
if __name__ == "__main__":
    wpilib.run(MyRobot)