from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import ctre
from wpilib._wpilib import Joystick
import wpimath.controller
import math

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        turnOffset = 16.787
        self.turnMotor = ctre.TalonFX(3)
        self.driveMotor = ctre.TalonFX(1)
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
        self.turnController = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        self.turnController.enableContinuousInput(-180, 180)
        self.previousTarget = 0
        self.initiateModule()
        self.directionReversed = False
        # self.turnMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        
    def initiateModule(self):
        self.turnMotor.setSelectedSensorPosition(int(self.absoluteEncoder.getAbsolutePosition() *2048*12.8/360))
        
    def zeroModule(self):
        pass
    
    def move(self):
        pass
    
    def teleopInit(self) -> None:
        self.joystick = wpilib.Joystick(0)
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
            self.turnController = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        self.diagnosticPeriodic()
        x = self.joystick.getX()
        y = self.joystick.getY()
        wpilib.SmartDashboard.putNumber("X:", x)
        wpilib.SmartDashboard.putNumber("Y:", y)
        if (x > 0.25 or x < -0.25) or (y > 0.25 or y < -0.25):
            angle = math.degrees(math.atan2(x, y))
            if angle > 180:
                angle -= 180
            self.previousTarget = angle
            wpilib.SmartDashboard.putNumber("targetAngle:", angle)
            self.turnOnly(angle)
            speed = math.hypot(x, y) / 4
            wpilib.SmartDashboard.putNumber("speed:", speed)
            if self.directionReversed:
                speed = -speed
            self.driveMotor.set(ctre.ControlMode.PercentOutput, speed)
        else:
            self.turnMotor.set(ctre.TalonFXControlMode.PercentOutput, 0)
            self.turnMotor.setNeutralMode(ctre.NeutralMode.Brake)
            self.driveMotor.set(ctre.ControlMode.PercentOutput, 0)
            wpilib.SmartDashboard.putNumber("speed:", 0)
            
            
    def diagnosticPeriodic(self):
        ''' Purpose is to provide diagnostics on the robot no matter the mode. '''
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        motorPosition = ((motorPosition % (2048*12.8)) * 360/(2048*12.8))
        if motorPosition > 180:
            motorPosition -= 360
        absolutePosition = self.absoluteEncoder.getAbsolutePosition()
        angleTolerance = 1
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
        wpilib.SmartDashboard.putNumber("modulePosition:", motorPosition)
        wpilib.SmartDashboard.putNumber("absolutePosition:", self.absoluteEncoder.getAbsolutePosition())
        motorPosition = self.optimize(motorPosition, angle)
        self.turnController.setSetpoint(angle)
        turnSpeed = self.turnController.calculate(motorPosition)
        '''turnDeadBand = 0.01
        if turnSpeed < turnDeadBand:
            turnSpeed = 0'''
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
        

if __name__ == "__main__":
    wpilib.run(MyRobot)
