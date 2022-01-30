import wpilib
import ctre
import rev

class Climber:
    def __init__(self, config = dict):
        self.config = config
        self.leftShoulder = rev.CANSparkMax(self.config["Climber"]["leftShoulder_ID"], rev.MotorType.kBrushless)
        self.leftShoulderEncoder = self.leftShoulder.getEncoder()
        self.rightShoulder = rev.CANSparkMax(self.config["Climber"]["rightShoulder_ID"], rev.MotorType.kBrushless)
        self.rightShoulderEncoder = self.rightShoulder.getEncoder()
        self.leftWinch = Winch(self.config["Climber"]["leftWinch_ID"], self.config["Climber"]["leftWinchLimitSwitch_DIO_ID"])
        self.rightWinch = Winch(self.config["Climber"]["rightWinch_ID"], self.config["Climber"]["rightWinchLimitSwitch_DIO_ID"])
        
        self.leftHook = wpilib.Solenoid(self.config["Climber"]["leftHook_PCM_ID"])
        self.rightHook = wpilib.Solenoid(self.config["Climber"]["rightHook_PCM_ID"])
    
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
        self.motor = ctre.TalonFX(falconID)
        self.limitSwitch = wpilib.DigitalInput(limitSwitchID) # digital input measures a voltage
        