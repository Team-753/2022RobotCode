import wpilib
import rev

class Intake:
    def __init__(self, config):
        self.carWash = rev.CANSparkMax(config["Intake"]["motorID"], rev.MotorType.kBrushless)
        self.lifter = wpilib.DoubleSolenoid(config["Intake"]["pistonID1"], config["Intake"]["pistonID2"])
        self.carWash.setIdleMode(rev.IdleMode.kBrake)

    def setCarWashSpeed(self, speed):
        self.carWash.set(speed)

    def setLifterUp(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kReverse)
    
    def setLifterDown(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kForward)
    
    def setLifterOff(self):
        self.lifter.set(wpilib.DoubleSolenoid.Value.kOff)

