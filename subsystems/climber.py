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
        self.leftWinch = Winch(self.config["Climber"]["leftWinch"]["ID"], self.config["Climber"]["leftWinch"]["Name"])
        self.rightWinch = Winch(self.config["Climber"]["rightWinch"]["ID"], self.config["Climber"]["rightWinch"]["Name"])
        
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

class Shoulder:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, sparkID, name):
        self.motor = rev.CANSparkMax(sparkID, rev.MotorType.kBrushless)
        self.currentLimit = 35 # This amperage limit has not been tested
        self.motor.setSelectedSensorPosition(0)
        self.position = self.motor.getSelectedSensorPosition()
        self.disabled = False
        self.name = name
    
    # WHERE I LAST LEFT OFF --------------------------------------------------------


class Winch:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, talonID, name):
        self.motor = ctre.TalonFX(talonID)
        self.currentLimit = 35 # This amperage limit has not been tested
        self.motor.setSelectedSensorPosition(0)
        self.position = self.motor.getSelectedSensorPosition()
        self.disabled = False
        self.name = name

    def release(self, speed):
        self.motor.set(ctre.ControlMode.PercentOutput, speed)
        self.disabled = False

    def retract(self, speed):
        self.checkEffort()
        if self.disabled == False:
            self.motor.set(ctre.ControlMode.PercentOutput, -speed)
            wpilib.SmartDashboard.putBool(self.name + " stopped", False)
        else:
            self.motor.set(ctre.ControlMode.PercentOutput, 0)
            self.motor.setNeutralMode(ctre.NeutralMode.Brake)
            wpilib.SmartDashboard.putBool(self.name + " stopped", True)

    def checkEffort(self):
        '''This checks if the winch is moving and how much current it is drawing.
        If it isn't moving and it is drawing too much current, the winch will be 'disabled'. '''
        current = self.motor.getStatorCurrent()
        velocity = self.motor.getSelectedSensorVelocity()
        wpilib.SmartDashboard.putNumber("Winch Sensor Velocity", velocity)
        if velocity < 0.1 and current > self.currentLimit:
            self.disabled = True
    
    def getPosition(self):
        self.position = self.motor.getSelectedSensorPosition()
        return(self.position)
