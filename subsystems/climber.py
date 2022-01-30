import wpilib
import ctre
import rev

class Climber:
    def __init__(self, config):
        self.leftShoulder = rev.CANSparkMax(15, rev.MotorType.kBrushless)
        self.rightShoulder = rev.CANSparkMax(16, rev.MotorType.kBrushless)
        self.leftWinch = Winch(13)
        self.rightWinch = Winch(14)
        
        self.leftHook = wpilib.Solenoid(0)
        self.rightHook = wpilib.Solenoid(1)
    
    def extendArms(self):
        ''''''
    def firstBarGrab(self):
        ''''''
    def nextBarGrab(self):
        ''''''
    def pullArms(self):
        ''''''
    def detachArms(self):
        ''''''
    def engageHooks(self):
        ''''''
    def disengageHooks(self):
        ''''''

class Winch:
    ''' I want this as its own class as it will need to be constantly updating its own variables and taking actions such as:
    - Current Draw
    - How much of the winch is let out
    - Asynchronously stopping the winch once its reached its limit or overcurrent'''
    def __init__(self, falconID, limitSwitchID) -> None:
        pass