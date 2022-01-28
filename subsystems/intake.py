import wpilib
import os
import rev

class Intake:
    def __init__(self, config):
        self.carWash = rev.CANSparkMax(17, rev.MotorType.kBrushless)
        self.lifter = wpilib.Solenoid(2)
    