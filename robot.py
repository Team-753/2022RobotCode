'''
GOALS:
    - Have an easily customizable autonomous mode that can take coordinate inputs and do those movements
        -- python GUI app that outputs vector data and autonomous "map" into a csv
        -- autonomous reads a csv
        -- Figure out how to actually go an exact distance without guessing & checking times
            --- Can be done using circumference of wheels
    - Working swerve drive
    - Configuration file (preferabbly json) for most if not all robot settings
    - Communicate with vision using networktables
    - Get motor framework down (assume we are using talonfx controllers)
    -- Nearly all completed
TODO:
- Not really anything as far as a bare bones only falcon drivetrain robot goes this is everything.
- Still need to integrate vision but that is hard when you do not even know what your robot will be doing
- Still need to integrate auxiliary files but yet again without any auxiliary systems as of yet, this is impossible

NOTE - Current Features:
- Autonomous including pathing and functions based on recorded session
- Easy drivetrain zeroing through smartdashboard
- Responsive drivetrain with PID control and functions such as coast and stationary
- easily expandible auxiliary and drivetrain functions
- easier to read and edit settings through the external config.json file

How to use:
- Teleoperated Mode:
  - Pretty self-explanatory
  
- Autonomous Mode:
  - Runs the pre-selected autonomous mode set in smartdashboard, the default is called "default"

- Test Mode:
  - Records an autonomous path and exports it --ON THE ROBORIO-- using datetime and is located in paths, would recommend using winscp
    to retrieve json

Installing Dependencies:
py -3.99 -m pip install robotpy robotpy-ctre robotpy-navx
'''
import wpilib
import json
import os
import driveTrain
import driverStation
from time import strftime, gmtime
from networktables import NetworkTables
import navx
import threading

cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True 
		cond.notify()

NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
vision = NetworkTables.getTable('aetherVision')

class MyRobot(wpilib.TimedRobot(period=0.02)):

    def robotInit(self):
        '''
        This function is called upon program startup and
        should be used for any initialization code.
        '''
        with open(f"{os.getcwd()}./config.json", "r") as f1:
            self.config = json.load(f1)
        self.driveTrain = driveTrain.driveTrain(self.config)
        self.driverStation = driverStation.driverStation(self.config)
        self.navx = navx.AHRS.create_spi()
        self.navx.reset()
        self.Timer = wpilib.Timer()

    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''
        autoPlanName = wpilib.SmartDashboard.getString("Auto Plan", "default")
        with open(f"{os.getcwd()}./paths/{autoPlanName}.json", 'r') as plan:  
            self.autoPlan = json.dump(plan)
        self.autonomousIteration = 0
        self.navx.reset()
        self.Timer.reset()
        
    
    def autonomousPeriodic(self):
        '''This function is called periodically during autonomous.'''
        # deadzones are already filtered so no reason to do any of that here
        if (self.autonomousIteration < len(self.autoPlan)): # to prevent any index out of range errors
            switches = self.autoPlan[self.autonomousIteration]
        if self.Timer.get() > self.config["matchSettings"]["autonomousTime"]:
            # auto is over
            self.Timer.stop()
            self.stopAll()
        else:
            self.switchActions(switches)
        self.autonomousIteration += 1
        self.driveTrain.refreshValues()

    def teleopInit(self):
        self.navx.reset()
        self.driveTrain.refreshValues()
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        switches = self.driverStation.checkSwitches()
        switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones(switches["driverX"], switches["driverY"], switches["driverZ"])
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
                with open(f"{os.getcwd()}./paths/{dt_gmt}.json", 'w') as path:
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
        if switchDict["playEasterEgg"]:
            self.driveTrain.easterEgg()
        if switchDict["zeroDriveTrainEncoders"]:
            self.driveTrain.zeroMotorEncoders()
    
    def evaluateDeadzones(self, x: float, y: float, z: float):
        if not (x > self.config["driverStation"]["joystickDeadZones"]["xDeadZone"]):
            x = 0
        if not (y > self.config["driverStation"]["joystickDeadZones"]["yDeadZone"]):
            y = 0
        if not (z > self.config["driverStation"]["joystickDeadZones"]["zDeadZone"]):
            z = 0
        return x, y, z
    
    def stopAll(self):
        ''' Exactly as it says, stops all of the functions of the robot '''
        self.driveTrain.stationary()
        # will add more as they come

if __name__ == "__main__":
    wpilib.run(MyRobot)
