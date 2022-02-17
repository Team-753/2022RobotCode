import wpilib
import os
import rev

class Intake:
    def __init__(self, config):
        self.carWash = rev.CANSparkMax(config["Intake"]["motorID"], rev.MotorType.kBrushless)
        self.lifter = wpilib.DoubleSolenoid(config["Intake"]["pistonID1"], config["Intake"]["pistonID2"])
        self.carWash.setIdleMode(rev.IdleMode.kBrake)

    def setCarWashSpeed(self, speed):
        self.carWash.set(speed)
    
    def setLifterPosition(self, position):
        if position == -1:
            self.lifter.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif position == 1:
            self.lifter.set(wpilib.DoubleSolenoid.Value.kForward)
        elif position == 0:
            self.lifter.set(wpilib.DoubleSolenoid.Value.kOff)