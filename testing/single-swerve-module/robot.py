from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import ctre
from wpilib._wpilib import Joystick
import wpilib.controller
import math

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        turnOffset = -20
        self.turnMotor = ctre.TalonFX(1)
        self.driveMotor = ctre.TalonFX(3)
        self.absoluteEncoder = ctre.CANCoder(2)
        self.absoluteEncoder.configMagnetOffset(turnOffset)
        #self.driveMotor = ctre.TalonFX(2)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
        #self.turnMotor.config
        self.kP = 0.005
        self.kI = 0.0025
        self.kD = 0
        self.turnController = wpilib.controller.PIDController(self.kP, self.kI, self.kD)
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
        self.turnMotor.setSelectedSensorPosition(self.absoluteEncoder.getAbsolutePosition() *2048*12.8/360)
        wpilib.SmartDashboard.putNumber("kP", self.kP)
        wpilib.SmartDashboard.putNumber("kI", self.kI)
        wpilib.SmartDashboard.putNumber("kD", self.kD)
    def teleopPeriodic(self) -> None:
        '''self.targetAngle = wpilib.SmartDashboard.getNumber("target:", 180)
        self.turnOnly(self.targetAngle)'''
        self.nkP = wpilib.SmartDashboard.getNumber("kP", self.kP)
        self.nkI = wpilib.SmartDashboard.getNumber("kI", self.kI)
        self.nkD = wpilib.SmartDashboard.getNumber("kD", self.kD)
        if self.nkP != self.kP or self.nkI != self.kI or self.nkD != self.nkD:
            self.kP = self.nkP
            self.kI = self.nkI
            self.kD = self.nkD
            self.turnController = wpilib.controller.PIDController(self.kP, self.kI, self.kD)
        self.diagnosticPeriodic()
        x = self.joystick.getX()
        y = self.joystick.getY()
        wpilib.SmartDashboard.putNumber("X:", x)
        wpilib.SmartDashboard.putNumber("Y:", y)
        if (x > 0.1 or x < -0.1) or (y > 0.1 or y < -0.1):
            angle = math.degrees(math.atan2(x, y))
            if angle > 180:
                angle -= 180
            self.previousTarget = angle
            wpilib.SmartDashboard.putNumber("angle:", angle)
            self.turnOnly(angle)
            speed = math.hypot(x, y) / 4
            wpilib.SmartDashboard.putNumber("speed:", speed)
            #self.driveMotor.set(ctre.ControlMode.PercentOutput, speed)
        else:
            self.turnOnly(self.previousTarget)
            #self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
            wpilib.SmartDashboard.putNumber("speed:", 0)
            
            
    def diagnosticPeriodic(self):
        ''' Purpose is to provide diagnostics on the robot no matter the mode. '''
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
        absolutePosition = self.absoluteEncoder.getAbsolutePosition()
        angleTolerance = 5
        # checking tolerances
        if motorPosition + angleTolerance > absolutePosition and motorPosition - angleTolerance < absolutePosition:
            wpilib.SmartDashboard.putBoolean("within range", True)
        else:
            wpilib.SmartDashboard.putBoolean("within range", False)
            
    def disabledPeriodic(self) -> None:
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
        wpilib.SmartDashboard.putNumber("internalPosition:", motorPosition)
        wpilib.SmartDashboard.putNumber("absolutePosition:", self.absoluteEncoder.getAbsolutePosition())
        wpilib.SmartDashboard.putNumber("RAW:", self.turnMotor.getSelectedSensorPosition(0))
        self.diagnosticPeriodic()
    
    def turnOnly(self, angle):
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
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
