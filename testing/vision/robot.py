import wpilib
from networktables import NetworkTables
import threading

cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True 
		cond.notify()

NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True) # this is also broken
sd = NetworkTables.getTable('SmartDashboard')

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        wpilib.CameraServer.launch()
        sd.putNumber("Test", 0)
        sd.putBoolean("robotEnabled", False)
        sd.putBoolean("aether", False)
        return super().robotInit()
    
    def disabledInit(self) -> None:
        sd.putBoolean("robotEnabled", False)
        sd.putBoolean("aether", False)
        return super().disabledInit()
    def disabledPeriodic(self) -> None:
        return super().disabledPeriodic()
    def teleopInit(self) -> None:
        sd.putBoolean("robotEnabled", True)
        return super().teleopInit()
    
    def autonomousInit(self) -> None:
        sd.putBoolean("aether", True)
        sd.putBoolean("robotEnabled", True)
        return super().autonomousInit()

if __name__ == "__main__":
    wpilib.run(MyRobot)