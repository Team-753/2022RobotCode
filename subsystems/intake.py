import wpilib
import rev

class Intake:
    def __init__(self, config):
        self.carWash = rev.CANSparkMax(config["Intake"]["motorID"], rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.lifter = wpilib.DoubleSolenoid(config["PCM"], wpilib.PneumaticsModuleType.CTREPCM, forwardChannel = config["Intake"]["pistonIDForward"], reverseChannel = config["Intake"]["pistonIDReverse"])
        self.intakeSpeed = config["Intake"]["IntakeSpeed"]
        self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.carWash.setInverted(config["Intake"]["inverted"])
        self.carWashTurnedOFF = True

    def carWashOn(self):
        self.carWash.set(self.intakeSpeed)
        
    def carWashReverse(self):
        self.carWash.set(-self.intakeSpeed)
        
    def carWashOff(self):
        self.carWash.set(0)
        self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

    def setLifterUp(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.carWashTurnedOFF = False
        print("lifter up")
        
    def setLifterDown(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kForward)
        self.carWashTurnedOFF = False
        print("lifter down")
        #self.carWashOn()
    
    def setLifterOff(self):
        if self.carWashTurnedOFF == False:
            self.lifter.set(wpilib.DoubleSolenoid.Value.kOff)
            self.carWashTurnedOFF = True
            print("lifter off")
        #self.carWash.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

