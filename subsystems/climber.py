import wpilib
import os
import ctre
import rev

class Climber:
    def __init__(self, config):
        self.leftShoulder = rev.CANSparkMax(15, rev.MotorType.kBrushless)
        self.rightShoulder = rev.CANSparkMax(16, rev.MotorType.kBrushless)
        self.leftWinch = ctre.TalonFX(13)
        self.rightWinch = ctre.TalonFX(14)
        
        self.leftHook = wpilib.Solenoid(0)
        self.rightHook = wpilib.Solenoid(1)
    
    def extendArms(self):
        
    def firstBarGrab(self):
        
    def nextBarGrab(self):
        
    def pullArms(self):
    
    def detachArms(self):
        
    def engageHooks(self):
        
    def disengageHooks(self):
        