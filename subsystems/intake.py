import wpilib
import rev

class Intake:
    def __init__(self, config):
        self.carWash = rev.CANSparkMax(config["Intake"]["motorID"], rev.MotorType.kBrushless)
        self.lifter = wpilib.DoubleSolenoid(config["Intake"]["pistonID1"], config["Intake"]["pistonID2"])
        self.intakeSpeed = config["Intake"]["IntakeSpeed"]
        self.carWash.setIdleMode(rev.IdleMode.kBrake)
        self.carWashDisabled = True

    def carWashOn(self):
        self.carWash.set(self.intakeSpeed)
        self.carWashDisabled = False
        
    def carWashReverse(self):
        self.carWash.set(-self.intakeSpeed)
        self.carWashDisabled = True
        
    def carWashOff(self):
        self.carWash.setIdleMode(rev.IdleMode.kCoast)
        self.carWashDisabled = True

    def setLifterUp(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.carWash.setIdleMode(rev.IdleMode.kBrake)
        
    def setLifterDown(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kForward)
        self.carWashOn()
    
    def setLifterOff(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kOff)
        self.carWash.setIdleMode(rev.IdleMode.kBrake)

