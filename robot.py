from typing import Tuple
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
        
        self.compressor = wpilib.Compressor(0)

    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''
        if self.autonomousMode == "dumb":
            autoPlanName = wpilib.SmartDashboard.getString("Auto Plan", "default")
            with open(f"{os.path.dirname(os.path.abspath(__file__))}/paths/{autoPlanName}.json", 'r') as plan:  
                self.autoPlan = json.load(plan)
            self.autonomousIteration = 0
            self.navx.reset()
            self.Timer.reset()
        elif self.autonomousMode == "smart":
            autoPlanName = wpilib.SmartDashboard.getString("Auto Plan", "default")
            self.autonomousController = autonomous.autonomous(autoPlanName)
            self.navx.reset()
            self.navx.resetDisplacement()
        
    
    def autonomousPeriodic(self):
        '''This function is called periodically during autonomous.'''
        # deadzones are already filtered so no reason to do any of that here
        if self.autonomousMode == "dumb":
            if (self.autonomousIteration < len(self.autoPlan)): # to prevent any index out of range errors
                switches = self.autoPlan[self.autonomousIteration] # this line currently breaks the code; indexing a dictionary
            if self.Timer.get() > self.config["matchSettings"]["autonomousTime"]:
                # auto is over
                self.Timer.stop()
                self.stopAll()
            else:
                self.switchActions(switches)
            self.autonomousIteration += 1
            self.driveTrain.refreshValues()
        elif self.autonomousMode == "smart":
            x, y, z, auxiliary = self.autonomousController.periodic(self.navx.getDisplacementX() * 39.37008, self.navx.getDisplacementY() * 39.37008) # need to eventually add support for auxiliary systems
            self.driveTrain.manualMove(x, y, z)

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
    
    def testInit(self):
        self.navx.reset()
        self.Timer.reset()
        self.autonomousSwitchList = []
        self.hasMoved = False
        wpilib.SmartDashboard.putBoolean("Recording", True)
    
    def testPeriodic(self):
        ''' Intended to record a path for autonomous '''
        switches = self.driverStation.checkSwitches()
        switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones(switches["driverX"], switches["driverY"], switches["driverZ"])
        if not self.hasMoved:
            if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
                self.hasMoved = True
                self.Timer.start()
        if self.hasMoved:
            if self.Timer.get() > self.config["matchSettings"]["autonomousTime"]:
                # autonomous recording period has ended
                self.recording = False
                self.Timer.stop()
                self.stopAll()
                wpilib.SmartDashboard.putBoolean("Recording", False)
                dt_gmt = strftime("%Y-%m-%d_%H:%M:%S", gmtime())
                with open(f"{os.path.dirname(os.path.abspath(__file__))}/paths/{dt_gmt}.json", 'w') as path:
                    path.write(json.dump(self.autonomousSwitchList))
            if self.recording:
                switches["time"] = self.Timer.get()
                self.autonomousSwitchList.append(switches) # adds 50x a second a little excessive maybe but needs to capture all inputs
                self.switchActions(switches)
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

    def evaluateDeadzones(self, inputs: Tuple):
        adjustedInputs = []
        for idx, input in enumerate(inputs):
            threshold = list(self.config["driverStation"]["joystickDeadZones"])[idx]
            adjustedValue = (abs(input) - threshold) / (1 - threshold)
            if input < 0 and adjustedValue != 0:
                adjustedValue = -adjustedValue
            adjustedInputs.append(adjustedValue)
        return adjustedInputs
    
    def nonEmergencyStop(self):
        ''' Exactly as it says, stops all of the functions of the robot '''
        self.driveTrain.stationary()
        # will add more as they come

if __name__ == "__main__":
    wpilib.run(MyRobot)
