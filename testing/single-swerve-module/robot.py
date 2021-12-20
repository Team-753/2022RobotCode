from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import ctre
from wpilib._wpilib import Joystick
import wpilib.controller
import math

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.absoluteoffset = 0
        self.previousAngle = 0
        self.turnMotor = ctre.TalonFX(0)
        self.absoluteEncoder = ctre.CANCoder(1)
        self.driveMotor = ctre.TalonFX(2)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.turnController = wpilib.controller.PIDController(0.05, 0.05, 0)
        self.turnController.enableContinuousInput(-180, 180)
        # self.turnMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        
    def initiateModule(self):
        #self.turnMotor.setSelectedSensorPosition((self.absoluteEncoder.getAbsolutePosition() - self.absoluteoffset) *)
        pass
        
    def zeroModule(self):
        pass
    
    def move(self):
        pass
    
    def teleopInit(self) -> None:
        self.joystick = wpilib.XboxController(0)
    
    def teleopPeriodic(self) -> None:
        x = self.joystick.Axis.kLeftX
        y = self.joystick.Axis.kLeftY
        if x > 0.25 and y > 0.25:
            angle = math.degrees(math.atan2(x, y))
            self.turnOnly(angle)
    
    def turnOnly(self, angle):
        motorPosition = self.turnMotor.getSelectedSensorPosition(0)
        self.turnController.setSetpoint(angle * 12.8)
        turnSpeed = self.turnController.calculate(motorPosition)
        self.turnMotor.set(ctre.ControlMode.PercentOutput, turnSpeed)
        

if __name__ == "__main__":
    wpilib.run(MyRobot)