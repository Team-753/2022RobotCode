import wpilib
import rev
import ctre
import wpimath.controller

class Shooter:
    def __init__(self, config = dict) -> None:
        self.motor = rev.CANSparkMax(config["Tower"]["shooterID"])
        
        self.kP = 0.005
        self.kI = 0
        self.kD = 0
        self.encoder = self.motor.getEncoder()
        self.shooterPID = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
        self.motor.setInverted(False)
    
    def setSpeed(self, velocity):
        '''
        This is in rpm.
        '''
        self.shooterPID.setSetpoint(self.encoder.getVelocity()) # getVelocity is in RPM
        speed = self.shooterPID.calculate(velocity)
        self.motor.set(speed)
    
    def getSpeed(self):
        return(self.encoder.getVelocity())
    
class Feeder:
    def __init__(self, config = dict) -> None:
        self.motor = ctre.VictorSPX(config["Tower"]["shooterFeederID"])
        self.motor.setInverted(False)
    
    def setSpeed(self, speed):
        self.motor.set(ctre.ControlMode.PercentOutput, speed)

class BallClimber:
    def __init__(self, config = dict) -> None:
        self.motor = ctre.VictorSPX(config["Tower"]["towerFeederID"])
        self.motor.setInverted(False)

