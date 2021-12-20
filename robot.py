from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import ctre
from wpilib._wpilib import Joystick
import wpilib.controller
import math

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.turnMotor = ctre.TalonFX(1)
        self.driveMotor = ctre.TalonFX(3)
        self.absoluteEncoder = ctre.CANCoder(2)
        self.absoluteEncoder.configMagnetOffset(-20)
        #self.driveMotor = ctre.TalonFX(2)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180)
        #self.turnMotor.config
        self.turnController = wpilib.controller.PIDController(0.005, 0.0025, 0)
        self.turnController.enableContinuousInput(-180, 180)
        self.previousTarget = 0
        self.initiateModule()
        # self.turnMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        
    def initiateModule(self):
        self.turnMotor.setSelectedSensorPosition(self.absoluteEncoder.getAbsolutePosition() *2048*12.8/360)
        
        
    def zeroModule(self):
        pass
    
    def move(self):
        pass
    
    def teleopInit(self) -> None:
        self.joystick = wpilib.Joystick(0)
    
    def teleopPeriodic(self) -> None:
        '''self.targetAngle = wpilib.SmartDashboard.getNumber("target:", 180)
        self.turnOnly(self.targetAngle)'''
        x = self.joystick.getX()
        y = self.joystick.getY()
        wpilib.SmartDashboard.putNumber("X:", x)
        wpilib.SmartDashboard.putNumber("Y:", y)
        if (x > 0.1 or x < -0.1) or (y > 0.1 or y < -0.1):
            angle = math.degrees(math.atan2(x, y))
            self.previousTarget = angle
            wpilib.SmartDashboard.putNumber("angle:", angle)
            self.turnOnly(angle)
            speed = math.hypot(x, y) / 4
            wpilib.SmartDashboard.putNumber("speed:", speed)
            self.driveMotor.set(ctre.ControlMode.PercentOutput, speed)
        else:
            self.turnOnly(self.previousTarget)
            self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
            wpilib.SmartDashboard.putNumber("speed:", 0)
            
    def disabledPeriodic(self) -> None:
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8)) -180
        wpilib.SmartDashboard.putNumber("internalPosition:", motorPosition)
        wpilib.SmartDashboard.putNumber("absolutePosition:", self.absoluteEncoder.getAbsolutePosition())
        wpilib.SmartDashboard.putNumber("RAW:", self.turnMotor.getSelectedSensorPosition(0))
    
    def turnOnly(self, angle):
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8)) - 180
        wpilib.SmartDashboard.putNumber("internalPosition:", motorPosition)
        wpilib.SmartDashboard.putNumber("absolutePosition:", self.absoluteEncoder.getAbsolutePosition())
        self.turnController.setSetpoint(angle)
        turnSpeed = self.turnController.calculate(motorPosition)
        '''turnDeadBand = 0.01
        if turnSpeed < turnDeadBand:
            turnSpeed = 0'''
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        

if __name__ == "__main__":
    wpilib.run(MyRobot)