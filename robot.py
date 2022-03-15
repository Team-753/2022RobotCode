from xmlrpc.client import FastMarshaller
import wpilib
import json
import os
from networktables import NetworkTables
import navx
import threading
from subsystems.driveTrain import driveTrain
#from controlsystems.autonomous import Autonomous
from controlsystems.driverStation import driverStation
from subsystems.climber import Climber
from subsystems.intake import Intake
from subsystems.driveTrain import driveTrain
from subsystems.tower import Tower
from controlsystems.autonomous2 import shootAndRun
from controlsystems.zPID import zPID

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
        camera = wpilib.CameraServer()
        camera.launch()
        
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
        wpilib.SmartDashboard.putNumber("NAVX OFFSET", 0)
        self.DEBUGSTATEMENTS = True
        self.revvingShooter = False
        #self.climber.zeroEncoders()
        #wpilib.SmartDashboard.putString("Play of the Game", "straightLine")
        
    def disabledInit(self) -> None:
        self.intake.carWashOff()
        self.tower.towerCoast()
        self.driveTrain.coast()
        self.angleOffset = wpilib.SmartDashboard.getNumber("NAVX OFFSET", 0)
        return super().disabledInit()

    def autoActions(self, action):
        actionName = action[0]
        if actionName == "revShooter":
            self.tower.shootVariable(action[1])
            self.auto.queuePosition += 1
        elif actionName == "wait":
            if self.Timer.get() > action[1]:
                self.waiting = False
                self.auto.queuePosition += 1
                self.Timer.stop()
                self.Timer.reset()
            elif self.waiting == False:
                self.waiting = True
                self.Timer.start()
        elif actionName == "done":
            pass
        elif actionName == "indexBall":
            self.tower.indexer()
            self.auto.queuePosition += 1
        elif actionName == "indexStop":
            self.tower.towerCoast()
            self.auto.queuePosition += 1
        elif actionName == "shooterOff":
            self.tower.coastShooter()
            self.auto.queuePosition += 1
        elif actionName == "intakeOn":
            self.intake.carWashOn()
            self.auto.queuePosition += 1
        elif actionName == "intakeDown":
            self.intake.setLifterDown()
            self.auto.queuePosition += 1
        elif actionName == "intakeOff":
            self.intake.carWashOff()
            self.auto.queuePosition += 1
        elif actionName == "move":
            if self.Timer.get() > action[2]:
                self.waiting = False
                self.auto.queuePosition += 1
                self.Timer.stop()
                self.Timer.reset()
            elif self.waiting == False:
                self.waiting = True
                self.Timer.start()
            else: 
                '''navxAngle = self.getNavx360()
                z = 0
                if navxAngle < (self.autoAngle + 2.5) and navxAngle > (self.autoAngle - 2.5):
                    z = 0
                elif navxAngle > self.autoAngle:
                    z = -0.2
                elif navxAngle < self.autoAngle:
                    z = 0.2'''
                self.driveTrain.move(action[1][0], action[1][1], 0)
        elif actionName == "stationary":
            self.driveTrain.stationary()
            self.auto.queuePosition += 1
        elif actionName == "turnTo":
            navxAngle = self.getNavx360()
            self.autoAngle = action[1]
            #print(f"Current Angle: {navxAngle}, Target Angle: {action[1]}")
            if navxAngle < (action[1] + 2.5) and navxAngle > (action[1] - 2.5):
                self.driveTrain.stationary()
                self.auto.queuePosition += 1
            elif navxAngle > action[1]:
                self.driveTrain.move(0, 0, -0.2)
            elif navxAngle < action[1]:
                self.driveTrain.move(0, 0, 0.2)
    
    def getNavxOneEighty(self):
        angle = self.navx.getAngle()
        angle %= 360
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        return angle
    
    def getNavx360(self):
        angle = self.navx.getAngle() - self.angleOffset
        angle %= 360
        return angle
            
    def testInit(self) -> None:
        self.climber.engageHooks()
        self.intake.setLifterUp()
        return super().testInit()
    
    def testPeriodic(self) -> None:
        return super().testPeriodic()
    
    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''
    #self.navx.reset()
        '''self.angleOffset = wpilib.SmartDashboard.getNumber("NAVX OFFSET", 0)
        #self.angleOffset = -45
        self.autoAngle = self.angleOffset
        self.navx.setAngleAdjustment(self.angleOffset)
        self.driveTrain.fieldOrient = False
        self.auto = shootAndRun()
        self.waiting = False
        self.Timer.reset()'''
        self.navx.reset()
        self.targetAngle = wpilib.SmartDashboard.getNumber("auto angle", 0)
        self.zPID = zPID(self.navx)
        

    def autonomousPeriodic(self):
        '''This function is called periodically during autonomous.'''
        '''actionToDo = self.auto.periodic()
        self.autoActions(actionToDo)'''
        z = self.zPID.periodic(self.targetAngle)
        if z != 0:
            self.driveTrain.move(0, 0, z) # TODO: Use this in teleop where if z input = 0 try to maintain previous robot heading
        else:
            self.driveTrain.stationary()
        
        

    def teleopInit(self):
        self.shoulderTargetAngle = 30
        self.climber.leftArm.shoulder.zeroShoulder()
        self.climber.rightArm.shoulder.zeroShoulder()
        self.driveTrain.fieldOrient = True
        
    def teleopPeriodic(self):
        '''This function is called periodically during operator control.'''
        switches = self.driverStation.checkSwitches()
        self.tower.getBallDetected()
        # wpilib.SmartDashboard.putNumber("winch position", self.climber.rightArm.winch.getRotations())
        switches["driverX"], switches["driverY"], switches["driverZ"], switches["moveArms"], switches["moveWinches"] = self.evaluateDeadzones((switches["driverX"], switches["driverY"], switches["driverZ"], switches["moveArms"], switches["moveWinches"]))
        
        self.switchActions(switches)
        
    def disabledPeriodic(self):
        pass
    
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
            self.tower.shootFromTarmac()
        elif switchDict["revShooterClose"]:
            self.tower.shootUpClose()
        else:
            self.tower.coastShooter()
            
        if switchDict["ballIndexerIn"]:
            #self.tower.prepareBall() 
            self.tower.indexer()
        elif switchDict["ballSystemOut"]:
            self.tower.reverse()
        else:
            self.tower.towerCoast()
            
        if switchDict["swerveAfterburners"]:
            self.driveTrain.swerveSpeedFactor = 0.75
        else:
            self.driveTrain.swerveSpeedFactor = self.config["RobotDefaultSettings"]["robotSpeedLimiter"]
        
        if switchDict["armStraightUp"]:
            pass
        elif switchDict["armToHooks"]:
            pass
        elif switchDict["armHome"]:
            self.climber.setArms(0, 18)
        elif switchDict["leftWinchIn"]:
            self.climber.leftArm.moveWinch(-0.5)
            self.climber.rightArm.brake()
        elif switchDict["rightWinchIn"]:
            self.climber.rightArm.moveWinch(-0.5)
            self.climber.leftArm.brake()
        elif switchDict["winchesIn"]:
            self.climber.leftArm.winch.setRPM(60)
            self.climber.rightArm.winch.setRPM(60)
        elif switchDict["winchesOut"]:
            self.climber.leftArm.winch.setRPM(-60)
            self.climber.rightArm.winch.setRPM(-60)
        else:
            self.climber.leftArm.winch.brake()
            self.climber.rightArm.winch.brake()
        if abs(switchDict["shoulderValue"]) > 0.1:
            if self.shoulderTargetAngle + switchDict["shoulderValue"] > 180 or self.shoulderTargetAngle + switchDict["shoulderValue"] < 30:
                pass
            else:
                self.shoulderTargetAngle += switchDict["shoulderValue"]
        self.climber.leftShoulder.setAngle(self.shoulderTargetAngle)
        self.climber.rightShoulder.setAngle(self.shoulderTargetAngle)
        '''elif switchDict["leftWinchOut"]:
            self.climber.leftArm.moveWinch(-0.5)
            self.climber.rightArm.brake()
        elif switchDict["rightWinchOut"]:
            self.climber.rightArm.moveWinch(-0.5)
            self.climber.leftArm.brake()'''
        if switchDict["tightenPeterHooks"]:
            self.climber.engageHooks()
        elif switchDict["releasePeterHooks"]:
            self.climber.disengageHooks()
    
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
