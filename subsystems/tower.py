import wpilib
import rev
import ctre
import wpimath.controller

class Tower:
    def __init__(self, config):
        self.shooterMotor = rev.CANSparkMax(config["Tower"]["shooterID"], rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.shooterEncoder = self.shooterMotor.getEncoder()

        self.feederMotor = ctre.VictorSPX(config["Tower"]["towerFeederID"])
        self.feederSpeed = 0.5

        self.ballClimberMotor = ctre.VictorSPX(config["Tower"]["ballClimberID"])
        self.ballClimberSpeed = 0.5

        self.proximitySensor = wpilib.AnalogInput(config["Tower"]["proximitySensorID"])

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
        self.ballClimberMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.feederMotor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def towerCoast(self):
        self.ballClimberMotor.setNeutralMode(ctre.NeutralMode.Coast)
        self.feederMotor.setNeutralMode(ctre.NeutralMode.Coast)
    
    def coastShooter(self):
        self.shooterMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        
    def brakeShooter(self):
        self.shooterMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
    
    def setBallClimberSpeed(self, speed):
        '''Sets the percent output of the tower's ball climber motor.'''
        self.ballClimberMotor.set(ctre.ControlMode.PercentOutput, speed)
    
    def getShooterVelocity(self):
        '''Returns velocity in RPM.'''
        return(self.shooterEncoder.getVelocity())
    
    def getBallDetected(self):
        '''This will return True or False (ball or no ball) when I figure out what the sensor values are.'''
        value = self.proximitySensor.getValue()
        wpilib.SmartDashboard.putNumber("Proximity Sensor", value)
        return(False)
    
    def prepareBall(self):
        self.setFeederSpeed(self.feederSpeed)
        if self.getBallDetected:
            self.setBallClimberSpeed(self.ballClimberSpeed)
        else:
            self.setBallClimberSpeed(0)
        
    def reverse(self):
        self.setFeederSpeed(-self.feederSpeed)
        self.setBallClimberSpeed(-self.ballClimberSpeed)

    def shoot(self, targetVelocity):
        '''Sets a target velocity for the shooter in RPM. 
        When it gets within the shooter's PID tolerance it turns on the feeder and ball climber.'''
        if abs(self.getShooterVelocity() - targetVelocity) < self.PIDTolerance:
            self.setFeederSpeed(self.feederSpeed)
            self.setBallClimberSpeed(self.ballClimberSpeed)
        else:
            self.setShooterVelocity(targetVelocity)