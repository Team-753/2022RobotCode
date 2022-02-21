from hal import setPWMPeriodScale
import wpilib
import rev
import ctre
import wpimath.controller

class Tower:
    def __init__(self, config):
        self.shooterMotor = rev.CANSparkMax(config["Tower"]["shooterID"])
        self.shooterEncoder = self.shooterMotor.getEncoder()

        self.feederMotor = ctre.VictorSPX(config["Tower"]["towerFeederID"])

        self.ballClimberMotor = ctre.VictorSPX(config["Tower"]["ballClimberID"])

        self.proximitySensor = wpilib.DigitalInput(config["Tower"]["proximitySensorID"])

        self.PIDTolerance = 42
        self.kP = 0.005
        self.kI = 0
        self.kD = 0
        self.PID = wpimath.controller.PIDController(self.kP, self.kI, self.kD)
    
    def setShooterVelocity(self, velocity):
        '''Sets the RPM of the shooter flywheel.'''
        self.PID.setSetpoint(self.shooterEncoder.getVelocity)
        self.shooterMotor.set(self.PID.calculate(velocity))
    
    def setFeederSpeed(self, speed):
        '''Sets the percent output of the tower's base feeder motor.'''
        self.feederMotor.set(ctre.ControlMode.PercentOutput, speed)
    
    def towerBrake(self):
        self.shooterMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.ballClimberMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.feederMotor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def towerCoast(self):
        self.shooterMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.ballClimberMotor.setNeutralMode(ctre.NeutralMode.Coast)
        self.feederMotor.setNeutralMode(ctre.NeutralMode.Coast)
    
    def setBallClimberSpeed(self, speed):
        '''Sets the percent output of the tower's ball climber motor.'''
        self.ballClimberMotor.set(ctre.ControlMode.PercentOutput, speed)
    
    def getShooterVelocity(self):
        '''Returns velocity in RPM.'''
        return(self.shooterEncoder.getVelocity())
    
    def getBallDetected(self):
        '''This will return True or False (ball or no ball) when I figure out what the sensor values are.'''
        value = self.proximitySensor.get()
        wpilib.SmartDashboard.putNumber("Proximity Sensor", value)
        return(False)
    
    def prepareBall(self):
        self.setFeederSpeed(1)
        if self.getBallDetected:
            self.setBallClimberSpeed(1)
        else:
            self.setBallClimberSpeed(0)
    
    def FIRE(self, targetVelocity):
        if abs(self.getShooterVelocity() - targetVelocity) < self.PIDTolerance:
            self.setFeederSpeed(1)
            self.setBallClimberSpeed(1)
        else:
            self.setShooterVelocity(targetVelocity)