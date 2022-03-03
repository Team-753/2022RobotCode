import wpilib
import rev

class Intake:
    def __init__(self, config):
        self.carWash = rev.CANSparkMax(config["Intake"]["motorID"], rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.lifter = wpilib.DoubleSolenoid(config["PCM"], wpilib.PneumaticsModuleType.CTREPCM, forwardChannel = config["Intake"]["pistonIDForward"], reverseChannel = config["Intake"]["pistonIDReverse"])
        self.intakeSpeed = config["Intake"]["IntakeSpeed"]
        self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.carWash.setInverted(config["Intake"]["inverted"])
        self.carWashDisabled = True

    def carWashOn(self):
        print("intake on")
        self.carWash.set(self.intakeSpeed)
        self.carWashDisabled = False
        
    def carWashReverse(self):
        self.carWash.set(-self.intakeSpeed)
        self.carWashDisabled = True
        
    def carWashOff(self):
        print("intake off")
        self.carWash.set(0)
        self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.carWashDisabled = True

    def setLifterUp(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        
    def setLifterDown(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kForward)
        #self.carWashOn()
    
    def setLifterOff(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kOff)
        #self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

