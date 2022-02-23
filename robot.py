import wpilib
import json
import os
from networktables import NetworkTables
import navx
import threading
from subsystems.driveTrain import driveTrain
from controlsystems.autonomous import Autonomous
from controlsystems.driverStation import driverStation
from subsystems.climber import Climber
from subsystems.intake import Intake
from subsystems.driveTrain import driveTrain
from subsystems.tower import Tower

'''cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True 
		cond.notify()

NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True) # this is also broken
vision = NetworkTables.getTable('aetherVision')'''

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        self.tower = Tower(self.config)
        self.intake = Intake(self.config)
        self.driverStation = driverStation(self.config)
        self.navx = navx.AHRS(wpilib._wpilib.I2C.Port.kOnboard, update_rate_hz=100)
        self.navx.reset()
        self.driveTrain = driveTrain(self.config, self.navx)
        self.Timer = wpilib.Timer()
        self.DEBUGSTATEMENTS = False
        

    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''
        pass

    def autonomousPeriodic(self):
        '''This function is called periodically during autonomous.'''
        pass

    def teleopInit(self):
        self.navx.reset() # NOTE: In production code get rid of this line (it will cause problems when autonomous moves the robot)
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        switches = self.driverStation.checkSwitches()
        switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones((switches["driverX"], switches["driverY"], switches["driverZ"]))
        self.switchActions(switches)
        if self.DEBUGSTATEMENTS:
            wpilib.SmartDashboard.putNumber("Proximity Sensor", self.tower.getProximitySensor())
        
    def disabledPeriodic(self):
        ''' Intended to update shuffleboard with drivetrain values used for zeroing '''
        if self.DEBUGSTATEMENTS:
            vals = self.driveTrain.refreshValues()
            wpilib.SmartDashboard.putNumber("FrontLeftAbs", vals[0][3])
            wpilib.SmartDashboard.putNumber("FrontRightAbs", vals[1][3])
            wpilib.SmartDashboard.putNumber("RearLeftAbs", vals[2][3])
            wpilib.SmartDashboard.putNumber("RearRightAbs", vals[3][3])
    
    def disabledInit(self) -> None:
        self.driveTrain.coast()
        self.tower.towerCoast()
        self.tower.coastShooter()
        self.intake.carWashOff()
    
    def switchActions(self, switchDict: dict):
        ''' Actually acts on and calls commands based on inputs from multiple robot modes '''
        if switchDict["driverX"] != 0 or switchDict["driverY"] != 0 or switchDict["driverZ"] != 0:
            self.driveTrain.manualMove(switchDict["driverX"], switchDict["driverY"], switchDict["driverZ"], switchDict["navxAngle"])
        else:
            self.driveTrain.stationary()
            
        if switchDict["swapFieldOrient"]:
            self.driveTrain.fieldOrient = not self.driveTrain.fieldOrient # swaps field orient to its opposite value
            wpilib.SmartDashboard.putBoolean("Field Orient", self.driveTrain.fieldOrient)
            
        if switchDict["resetDriveTrainEncoders"]:
            self.driveTrain.reInitiateMotorEncoders()
            
        if switchDict["intakeUp"]:
            self.intake.setLifterUp()
            
        if switchDict["intakeDown"]:
            self.intake.setLifterDown()
            
        if switchDict["intakeOn"]:
            self.intake.carWashOn()
            
        if switchDict["intakeOff"]:
            self.intake.carWashOff() 
            
        if switchDict["revShooter"]:
            self.tower.shoot(5000) # this is temporary until we can start getting vision data
        else:
            self.tower.coastShooter()
            
        if switchDict["ballIndexerIn"]:
            self.tower.prepareBall()   
        elif switchDict["ballSystemOut"]:
            self.tower.reverse()
            self.intake.carWashReverse()
        else:
            self.tower.towerCoast()
            
        if not self.intake.carWashDisabled:
            self.intake.carWashOn()
        
    def evaluateDeadzones(self, inputs):
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = self.config["driverStation"]["joystickDeadZones"][(list(self.config["driverStation"]["joystickDeadZones"])[idx])]
            if abs(input) > threshold: 
                adjustedValue = (abs(input) - threshold) / (1 - threshold)
                if input < 0 and adjustedValue != 0:
                    adjustedValue = -adjustedValue
            else:
                adjustedValue = 0
            adjustedInputs.append(adjustedValue)
        return adjustedInputs[0], adjustedInputs[1], adjustedInputs[2]
    
    def nonEmergencyStop(self):
        ''' Exactly as it says, stops all of the functions of the robot '''
        self.driveTrain.stationary()
        self.tower.towerCoast()
        self.intake.carWashOff()
        self.tower.coastShooter()

if __name__ == "__main__":
    wpilib.run(MyRobot)
