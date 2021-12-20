import wpilib
import rev 
# assume we are using spark max controllers

class neoAuxiliary:
    def __init__(self, config: dict):
        self.motor = rev.CANSparkMax(config["auxiliarySystems"]["neoAuxiliary"]["motorID"], rev.CANSparkMax.MotorType.kBrushless)
        self.motorEncoder = self.motor.getEncoder()
    
    def spin(self, speed):
        ''' Takes in a value between -1 -> +1 '''
        self.motor.set(speed)
        
    def stop(self):
        self.motor.set(0)
    
    def brake(self):
        self.motor.setIdleMode(rev.IdleMode.kBrake)
    
    def coast(self):
        self.motor.setIdleMode(rev.IdleMode.kCoast)
        
    def getValues(self):
        return self.motorEncoder.getVelocity() # Alternatively use getPosition()
