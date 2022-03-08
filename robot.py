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

cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True 
		cond.notify()

NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True) # this is also broken
smartDash = NetworkTables.getTable('SmartDashboard')

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        wpilib.CameraServer.launch()
        
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)
        self.tower = Tower(self.config)
        self.intake = Intake(self.config)
        self.climber = Climber(self.config)
        self.driverStation = driverStation(self.config)
        self.navx = navx.AHRS.create_spi()
        self.driveTrain = driveTrain(self.config, self.navx)
        self.Timer = wpilib.Timer()
        self.DEBUGSTATEMENTS = True
        smartDash.putBoolean("robotEnabled", False)
        smartDash.putBoolean("aether", False)
        self.revvingShooter = False
        #self.climber.zeroEncoders()
        wpilib.SmartDashboard.putString("Play of the Game", "straightLine")
        wpilib.SmartDashboard.putNumber("targetRPM", 1)
        
        self.autonomousSwitches = {
            "intakeOn": self.intake.carWashOn(),
            "intakeOff": self.intake.carWashOff(),
            "intakeReverse": self.intake.carWashReverse(),
            "lowerIntake": self.intake.setLifterDown(),
            "raiseIntake": self.intake.setLifterUp(),
            "revShooter": self.smartAutoShooter(), # TODO: Change this later if not using vision maybe make a self variable that can be passed in to guage the distance to the hub
            "coastShooter": self.tower.coastShooter(),
            "indexBall": self.tower.indexer(),
            "coastBallPath": self.tower.towerCoast()
        }
        
    def disabledExit(self) -> None:
        smartDash.putBoolean("robotEnabled", True)
        
        
    def disabledInit(self) -> None:
        self.intake.carWashOff()
        self.tower.towerCoast()
        self.driveTrain.coast()
        smartDash.putBoolean("robotEnabled", False)
        smartDash.putBoolean("aether", False)
        return super().disabledInit()

    def smartAutoShooter(self):
        self.revvingShooter = not(self.revvingShooter)
    
    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''
        self.revvingShooter = False
        self.navx.reset()
        self.auto = Autonomous(wpilib.SmartDashboard.getString("Play of the Game", "straightLine"), self.navx)

    def autonomousPeriodic(self):
        '''This function is called periodically during autonomous.'''
        self.driveTrain.updateOdometry()
        pose = self.driveTrain.getFieldPosition()
        x, y, z, switches = self.auto.periodic(pose)
        print(x, y, z)
        '''if x == 0 and y == 0 and z ==0:
            self.driveTrain.coast()
        else:
            self.driveTrain.move(x, y, z)'''
        

    def teleopInit(self):
        self.navx.reset() # NOTE: In production code get rid of this line (it will cause problems when autonomous moves the robot)
        self.driveTrain.resetOdometry()
        self.navx.resetDisplacement()
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        switches = self.driverStation.checkSwitches()
        self.driveTrain.updateOdometry()
        pose = self.driveTrain.getFieldPosition()
        wpilib.SmartDashboard.putNumber("x", -pose[0])
        wpilib.SmartDashboard.putNumber("y", -pose[1])
        wpilib.SmartDashboard.putNumber("navX x", self.navx.getDisplacementX())
        wpilib.SmartDashboard.putNumber("navX y", self.navx.getDisplacementY())
        switches["driverX"], switches["driverY"], switches["driverZ"], switches["moveArms"], switches["moveWinches"] = self.evaluateDeadzones((switches["driverX"], switches["driverY"], switches["driverZ"], switches["moveArms"], switches["moveWinches"]))
        
        self.switchActions(switches)
        
    def disabledPeriodic(self):
        ''' Intended to update shuffleboard with drivetrain values used for zeroing '''
        if self.DEBUGSTATEMENTS:
            '''vals = self.driveTrain.refreshValues()
            smartDash.putNumber("FrontLeftAbs", vals[0][3])
            smartDash.putNumber("FrontRightAbs", vals[1][3])
            smartDash.putNumber("RearLeftAbs", vals[2][3])
            smartDash.putNumber("RearRightAbs", vals[3][3])'''
            self.tower.getBallDetected()
            wpilib.SmartDashboard.putNumber("NavX Angle", self.navx.getAngle())
            wpilib.SmartDashboard.putNumber("NavX X", self.navx.getRawGyroX())
            wpilib.SmartDashboard.putNumber("NavX Y", self.navx.getRawGyroY())
            wpilib.SmartDashboard.putNumber("NavX Z", self.navx.getRawGyroZ())
    
    def disabledInit(self) -> None:
        self.driveTrain.coast()
        self.tower.towerCoast()
        self.tower.coastShooter()
        self.intake.carWashOff()
    
    def switchActions(self, switchDict: dict):
        ''' Actually acts on and calls commands based on inputs from multiple robot modes '''
        if switchDict["driverX"] != 0 or switchDict["driverY"] != 0 or switchDict["driverZ"] != 0:
            self.driveTrain.move(switchDict["driverX"], switchDict["driverY"], switchDict["driverZ"])
        else:
            self.driveTrain.stationary()
            
        if switchDict["swapFieldOrient"]:
            #self.driveTrain.fieldOrient = not self.driveTrain.fieldOrient # swaps field orient to its opposite value
            #smartDash.putBoolean("Field Orient", self.driveTrain.fieldOrient)
            self.navx.reset()
            
        if switchDict["resetDriveTrainEncoders"]:
            self.driveTrain.reInitiateMotorEncoders()
            
        if switchDict["intakeUp"]:
            self.intake.setLifterUp()
            
        if switchDict["intakeDown"]:
            self.intake.setLifterDown()
            
        if switchDict["intakeOn"]:
            self.intake.carWashOn()
        elif switchDict["ballSystemOut"]:
            self.intake.carWashReverse()
        else:
            self.intake.carWashOff()
            
        if switchDict["revShooter"]:
            smartDash.putBoolean("aether", True)
            self.tower.setShooterVelocity(wpilib.SmartDashboard.getNumber("targetRPM", 1)) # this is temporary until we can start getting vision data
        else:
            smartDash.putBoolean("aether", False)
            self.tower.coastShooter()
            
        if switchDict["ballIndexerIn"]:
            #self.tower.prepareBall() 
            self.tower.indexer()
        elif switchDict["ballSystemOut"]:
            self.tower.reverse()
        else:
            self.tower.towerCoast()
            
        if switchDict["swerveAfterburners"]:
            self.driveTrain.swerveSpeedFactor = 1
        else:
            self.driveTrain.swerveSpeedFactor = self.config["RobotDefaultSettings"]["robotSpeedLimiter"]
        
        if switchDict["winchIn"]:
            self.climber.moveShoulders(0.25)
        elif switchDict["winchOut"]:
            self.climber.moveShoulders(-0.25)
        else:
            self.climber.brakeArms()
        
        if self.driverStation.climbModeActivated:
            self.climber.moveShoulders(switchDict["moveArms"])
            self.climber.moveWinches(switchDict["moveWinches"])

            if switchDict["releasePeterHooks"]:
                self.climber.disengageHooks()
            elif switchDict["tightenPeterHooks"]:
                self.climber.engageHooks()
    
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
        return adjustedInputs
    
    def nonEmergencyStop(self):
        ''' Exactly as it says, stops all of the functions of the robot '''
        self.driveTrain.stationary()
        self.tower.towerCoast()
        self.intake.carWashOff()
        self.tower.coastShooter()

if __name__ == "__main__":
    wpilib.run(MyRobot)
