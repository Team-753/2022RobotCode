import wpilib
import ctre

class falconAuxiliary:
    def __init__(self, config: dict):
        self.motor = ctre.TalonFX(config["auxiliarySystems"]["falconAuxiliary"]["motorID"])
    
    def spin(self, speed):
        ''' Takes in a value between -1 -> +1 '''
        self.motor.set(ctre.TalonFXControlMode.PercentOutput, speed)
        
    def stop(self):
        self.motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
    
    def brake(self):
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)
    
    def coast(self):
        self.motor.setNeutralMode(ctre.NeutralMode.Coast)
        
    def getValues(self):
        return self.motor.getSelectedSensorVelocity() # Alternatively use getSelectedSensorPosition()
    