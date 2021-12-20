from ctre._ctre import AbsoluteSensorRange, SensorInitializationStrategy
import wpilib
import ctre

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.absoluteoffset = 0
        self.turnMotor = ctre.TalonFX(0)
        self.absoluteEncoder = ctre.CANCoder(1)
        self.driveMotor = ctre.TalonFX(2)
        self.absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        self.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        self.turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180)
        # self.turnMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition)
        
    def initiateModule(self):
        #self.turnMotor.setSelectedSensorPosition((self.absoluteEncoder.getAbsolutePosition() - self.absoluteoffset) *)
        pass
        
    def zeroModule(self):
        pass
        

if __name__ == "__main__":
    wpilib.run(MyRobot)