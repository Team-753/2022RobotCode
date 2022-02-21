#trying to use github

import wpilib
import ctre
import os
import random
import math

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.falconList = [ctre.TalonFX(1), ctre.TalonFX(2)]
        for motor in self.falconList:
            motor.configFactoryDefault()
            motor.configurePID((0.05, 0.05, 0))
            motor.configIntegratedSensorAbsoluteRange(-180, 180)
        self.falconControlMode = ctre.ControlMode.Position
        
    
    def teleopInit(self) -> None:
        self.joystick = wpilib.Joystick(0)
        self.testGearRatio = 1
    
    def teleopPeriodic(self) -> None:
        x = self.joystick.getZ()
        y = self.joystick.getY()
        if x > 0.25 or y > 0.25:
            angle = math.degrees(math.atan(y/x))
            for motor in self.falconList:
                motor.set(self.falconControlMode, angle * self.testGearRatio)
        else:
            for motor in self.falconList:
                motor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def testInit(self) -> None:
        self.orchestra = ctre.Orchestra()
        for motor in self.falconList:
            self.orchestra.addInstrument(motor)
        self.orchestra.loadMusic(f"{os.getcwd()}./tunes/{random.randrange(0, 4)}.chrp")
        self.orchestra.play()
    
    def testPeriodic(self) -> None:
        pass
    
if __name__ == "__main__":
    wpilib.run(MyRobot)
