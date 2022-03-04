from operator import truediv
import wpilib
import rev
import ctre
import wpimath.controller

class Tower:
    def __init__(self, config):
        self.shooterMotor = rev.CANSparkMax(config["Tower"]["shooterID"], rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.shooterEncoder = self.shooterMotor.getEncoder()
        self.shooterMotor.setInverted(config["Tower"]["shooterInverted"])

        self.feederMotor = ctre.VictorSPX(config["Tower"]["towerFeederID"])
        self.feederSpeed = 1

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
        '''self.PID.setSetpoint(velocity)
        self.shooterMotor.set(self.PID.calculate(self.shooterEncoder.getVelocity()))'''
        self.shooterMotor.set(1)
    
    def setFeederSpeed(self, speed):
        '''Sets the percent output of the tower's base feeder motor.'''
        self.feederMotor.set(ctre.ControlMode.PercentOutput, speed)
    
    def towerBrake(self):
        self.ballClimberMotor.setNeutralMode(ctre.NeutralMode.Brake)
        self.feederMotor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def towerCoast(self):
        self.setFeederSpeed(0)
        self.setBallClimberSpeed(0)
        self.ballClimberMotor.setNeutralMode(ctre.NeutralMode.Coast)
        self.feederMotor.setNeutralMode(ctre.NeutralMode.Coast)
    
    def coastShooter(self):
        self.shooterMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.shooterMotor.set(0)
        
    def brakeShooter(self):
        self.shooterMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
    
    def setBallClimberSpeed(self, speed):
        '''Sets the percent output of the tower's ball climber motor.'''
        self.ballClimberMotor.set(ctre.ControlMode.PercentOutput, speed)
    
    def getShooterVelocity(self):
        '''Returns velocity in RPM.'''
        velocity = self.shooterEncoder.getVelocity()
        wpilib.SmartDashboard.putNumber("Flywheel Velocity", velocity)
        return(velocity)
    
    def getBallDetected(self):
        '''This will return True or False (ball or no ball) when I figure out what the sensor values are.'''
        value = self.proximitySensor.getValue()
        if value > 1000:
            ballAtTop = True
        else:
            ballAtTop = False
        wpilib.SmartDashboard.putNumber("distance sensor", value)
        wpilib.SmartDashboard.putBoolean("Ball At Top", ballAtTop)
        return ballAtTop
    
    def prepareBall(self):
        self.setFeederSpeed(self.feederSpeed)
        if self.getBallDetected:
            self.setBallClimberSpeed(self.ballClimberSpeed)
        else:
            self.setBallClimberSpeed(0)
            
    def runAllNoConsequences(self):
        self.setFeederSpeed(self.feederSpeed)
        self.setBallClimberSpeed(self.ballClimberSpeed)
        
    def reverse(self):
        self.setFeederSpeed(-self.feederSpeed)
        self.setBallClimberSpeed(-self.ballClimberSpeed)
    
    def flywheelUpToSpeed(self):
        if self.getShooterVelocity() > 4800:
            return True
        else:
            return False
    
    def indexer(self):
        ''' The main function and logic that is run through when pressing the auxiliary index button '''
        if self.flywheelUpToSpeed():
            self.runAllNoConsequences()
        elif self.getBallDetected():
            self.setFeederSpeed(self.feederSpeed)
        else:
            print("indexing all")
            self.setBallClimberSpeed(self.ballClimberSpeed)
            self.setFeederSpeed(self.feederSpeed)  

    def shoot(self, targetVelocity):
        '''Sets a target velocity for the shooter in RPM. 
        When it gets within the shooter's PID tolerance it turns on the feeder and ball climber.'''
        if abs(self.getShooterVelocity() - targetVelocity) < self.PIDTolerance:
            self.setFeederSpeed(self.feederSpeed)
            self.setBallClimberSpeed(self.ballClimberSpeed)
        else:
            self.setShooterVelocity(targetVelocity)