import wpilib
import json
import os
from time import strftime, gmtime
from networktables import NetworkTables
import navx
import threading
from subsystems.driveTrain import driveTrain
from controlsystems.autonomous import autonomous
from controlsystems.driverStation import driverStation
from subsystems.climber import Climber
from subsystems.intake import Intake
from subsystems.driveTrain import driveTrain
from subsystems.tower import Shooter, Feeder, ballClimber

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
        self.driveTrain = driveTrain(self.config)
        self.driverStation = driverStation(self.config)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()
        self.Timer = wpilib.Timer()
        self.autonomousMode = "smart" # alternatively "dumb"
        

    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''

    def autonomousPeriodic(self):
        '''This function is called periodically during autonomous.'''

    def teleopInit(self):
        self.navx.reset() # NOTE: In production code get rid of this line
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        switches = self.driverStation.checkSwitches()
        switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones((switches["driverX"], switches["driverY"], switches["driverZ"]))
        self.switchActions(switches)
        
    def disabledPeriodic(self):
        ''' Intended to update shuffleboard with drivetrain values used for zeroing '''
        self.driveTrain.refreshValues()
            
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
        # will add more as they come

if __name__ == "__main__":
    wpilib.run(MyRobot)
