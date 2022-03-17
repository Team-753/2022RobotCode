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

# ! This is important for networktables. I just copied it.
cond = threading.Condition()
notified = False
def connectionListener(connected, info):
	print(info, '; Connected=%s' % connected)
	with cond:
		notified = True
		cond.notify()

# ! I think this is instantiating networktables.
NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True) # this is also broken
smartDash = NetworkTables.getTable('SmartDashboard')

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # ! This is camera code copied from J=oe.
        camera = wpilib.CameraServer()
        camera.launch()
        
        # ! This allows the code to use the config.json file as a normal dictionary.
        folderPath = os.path.dirname(os.path.abspath(__file__))
        filePath = os.path.join(folderPath, 'config.json')
        with open (filePath, "r") as f1:
            self.config = json.load(f1)

        # ! Instantiating all the other files in the controlsystems and subsystems folders.
        self.tower = Tower(self.config)
        self.intake = Intake(self.config)
        self.climber = Climber(self.config)
        self.driverStation = driverStation(self.config)
        self.navx = navx.AHRS.create_spi()
        self.driveTrain = driveTrain(self.config, self.navx)
        
        # ! This timer is what Joe uses in his autonomous.
        self.Timer = wpilib.Timer()
        wpilib.SmartDashboard.putNumber("NAVX OFFSET", 0)
        
        # ! This turns on all the smartdashboard and print statements.
        self.DEBUGSTATEMENTS = True
        
        # ! This currently has no uses so I don't know what it is for.
        self.revvingShooter = False
        
        # ! This are commented out and I don't know why.
        #self.climber.zeroEncoders()
        
        # ! I don't know why this is important.
        #wpilib.SmartDashboard.putString("Play of the Game", "straightLine")
  
    def disabledInit(self) -> None:
        # ! This coasts all the motors (except the climber) when disabled. It sets the NAVX angle offset to whatever it is on SmartDashboard for some reason.
        self.intake.carWashOff()
        self.tower.towerCoast()
        self.driveTrain.coast()
        self.angleOffset = wpilib.SmartDashboard.getNumber("NAVX OFFSET", 0)
        # ! I don't know what this does.
        return super().disabledInit()

    
    # ! This is Joe's improvised autonomous.
    def autoActions(self, action):
        '''This method takes a list of tuples and strings. 
        Each tuple contains a string with the name of an action, and then a number if that action needs one.'''
        
        actionName = action[0]
        if actionName == "revShooter":
            self.tower.shootVariable(action[1])
            
        elif actionName == "wait":
            if self.Timer.get() > action[1]:
                self.waiting = False
                self.Timer.stop()
                self.Timer.reset()

            elif self.waiting == False:
                self.waiting = True
                self.Timer.start()

        elif actionName == "done":
            pass

        elif actionName == "indexBall":
            self.tower.indexer()
            
        elif actionName == "indexStop":
            self.tower.towerCoast()
            
        elif actionName == "shooterOff":
            self.tower.coastShooter()
            
        elif actionName == "intakeOn":
            self.intake.carWashOn()
            
        elif actionName == "intakeDown":
            self.intake.setLifterDown()
            
        elif actionName == "intakeOff":
            self.intake.carWashOff()
            
        elif actionName == "move":
            if self.Timer.get() > action[2]:
                self.waiting = False
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
            
        elif actionName == "turnTo":
            navxAngle = self.getNavx360()
            self.autoAngle = action[1]
            #print(f"Current Angle: {navxAngle}, Target Angle: {action[1]}")
            if navxAngle < (action[1] + 2.5) and navxAngle > (action[1] - 2.5):
                self.driveTrain.stationary()
                
            elif navxAngle > action[1]:
                self.driveTrain.move(0, 0, -0.2)

            elif navxAngle < action[1]:
                self.driveTrain.move(0, 0, 0.2)

        else:
            self.auto.queuePosition -= 1 # This makes the queue position go up only if an action occurs.
        self.auto.queuePosition += 1



    def getNavxOneEighty(self):
        # ! This converts numbers in the interval 0 to 360 and changes them to fit the interval -180 to 180. 
        # ! This helps convert navx angle into something the math module can use.
        angle = self.navx.getAngle()
        angle %= 360
        if angle < -180:
            angle += 360
        elif angle > 180:
            angle -= 360
        return angle
    
    def getNavx360(self):
        # ! This gets the angle of the navx without the extra rotations.
        angle = self.navx.getAngle() - self.angleOffset
        angle %= 360
        return angle
            


    def testInit(self) -> None:
        # ! I think Joe used this to set up the robot for autonomous. 
        self.climber.engageHooks()
        self.intake.setLifterUp()
        return super().testInit()
    
    def testPeriodic(self) -> None:
        # ! I don't know what this does.
        return super().testPeriodic()
    


    def autonomousInit(self):
        '''This function is run once each time the robot enters autonomous mode.'''

        '''self.angleOffset = wpilib.SmartDashboard.getNumber("NAVX OFFSET", 0)
        self.angleOffset = -45
        self.autoAngle = self.angleOffset
        self.navx.setAngleAdjustment(self.angleOffset)
        self.driveTrain.fieldOrient = False
        self.auto = shootAndRun()
        self.waiting = False
        self.Timer.reset()'''

        # ! This resets navx and takes the angle from SmartDashboard and uses it as a parameter in a PID object in the controlsystems folder.
        # ! The zPID is for the turning during autonomous.
        self.navx.reset()
        self.targetAngle = wpilib.SmartDashboard.getNumber("auto angle", 0)
        self.zPID = zPID(self.navx)

    def autonomousPeriodic(self):
        '''This function is called every time the code runs during autonomous.'''
        
        # ! Joe's improvised autonomous.
        #actionToDo = self.auto.periodic()
        #self.autoActions(actionToDo)

        # ! This uses the method from the zPID object to calculate the z value based on the target angle defined in the init.
        # ! This method is incomplete.
        z = self.zPID.periodic(self.targetAngle)
        if z != 0:
            self.driveTrain.move(0, 0, z) # TODO: Use this in teleop where if z input = 0 try to maintain previous robot heading
        else:
            self.driveTrain.stationary()
        


    def teleopInit(self):
        # ! This sets the target angle for the shoulders. Since they start at 30 degrees the target angle starts at 30 degrees.
        self.shoulderTargetAngle = 30

        # ! This zeroes the shoulder encoders.
        self.climber.leftArm.shoulder.zeroShoulder()
        self.climber.rightArm.shoulder.zeroShoulder()

        # ! Field orient means the robot controls are relative to the field.
        self.driveTrain.fieldOrient = True
        
    def teleopPeriodic(self):
        '''This function is called every time the code runs during operator control.'''
        
        # ! This switches variable is a dictionary containing switch values, which are either booleans or floats.
        # ! The default values are written inside the getSwitches() method.
        switches = self.driverStation.checkSwitches()

        # ! This method checks the analog value from the IR sensor and determines whether it is small enough for a ball to be there.
        self.tower.getBallDetected()

        # wpilib.SmartDashboard.putNumber("winch position", self.climber.rightArm.winch.getRotations())

        # ! This changes these 5 switch values based on whether or not they are inside their respective deadzones.
        # ! If they are not they stay the same. If they are, that specific value becomes 0.
        switches["driverX"], switches["driverY"], switches["driverZ"], switches["moveArms"], switches["moveWinches"] = self.evaluateDeadzones((switches["driverX"], switches["driverY"], switches["driverZ"], switches["moveArms"], switches["moveWinches"]))
        
        # ! This method takes in the switch values and turns that into robot actions.
        self.switchActions(switches)
        


    def disabledPeriodic(self):
        # ! I want to put code here but I know I shouldn't.
        pass
    
    def disabledInit(self) -> None:
        # ! This coasts all the motors (but not the climber motors) so the robot is easier to move around.
        self.driveTrain.coast()
        self.tower.towerCoast()
        self.tower.coastShooter()

        # ! This turns the piston off on the intake but it does not release the pressure.
        self.intake.carWashOff()
    


    def switchActions(self, switchDict: dict):
        '''Actually acts on and calls commands based on inputs from multiple robot modes.
        The parameter switchDict comes from the method checkSwitches in the file driverStation.py.
        Returns nothing.'''

        # ! This is where the robot is told to drive
        if switchDict["driverX"] != 0 or switchDict["driverY"] != 0 or switchDict["driverZ"] != 0:
            self.driveTrain.move(switchDict["driverX"], switchDict["driverY"], switchDict["driverZ"])
        else:
            self.driveTrain.stationary()
        
        # ! This is where field orient is toggled. The navx is reset so toggling will also change the direction of field orient.
        if switchDict["swapFieldOrient"]:
            #self.driveTrain.fieldOrient = not self.driveTrain.fieldOrient # swaps field orient to its opposite value
            #smartDash.putBoolean("Field Orient", self.driveTrain.fieldOrient)
            self.navx.reset()
        
        # ! This resets the zeroes on the drivetrain.
        if switchDict["resetDriveTrainEncoders"]:
            self.driveTrain.reInitiateMotorEncoders()
        
        # ! This controls the piston on the intake.
        if switchDict["intakeUp"]:
            self.intake.setLifterUp()
        if switchDict["intakeDown"]:
            self.intake.setLifterDown()
        
        # ! This spins the intake motor.
        if switchDict["intakeOn"]:
            self.intake.carWashOn()
        elif switchDict["ballSystemOut"]:
            self.intake.carWashReverse()
        else:
            self.intake.carWashOff()
        
        # ! This revs up the shooter different amounts based on which method is used.
        if switchDict["revShooter"]:
            self.tower.shootFromTarmac()
        elif switchDict["revShooterClose"]:
            self.tower.shootUpClose()
        else:
            self.tower.coastShooter()
        
        # ! This spins the two motors in the tower that feeds the ball to the shooter.
        if switchDict["ballIndexerIn"]:
            #self.tower.prepareBall() 
            self.tower.indexer()
        elif switchDict["ballSystemOut"]:
            self.tower.reverse()
        else:
            self.tower.towerCoast()
        
        # ! This sets the speed multiplier of the drivetrain to a different value. The else statement sets it to default.
        if switchDict["swerveAfterburners"]:
            self.driveTrain.swerveSpeedFactor = 0.75
        else:
            self.driveTrain.swerveSpeedFactor = self.config["RobotDefaultSettings"]["robotSpeedLimiter"]
        

        # ! These methods control the arms.
        if switchDict["armStraightUp"]:
            pass

        elif switchDict["armToHooks"]:
            pass

        elif switchDict["armHome"]:
            # ! This method uses inverse kinematics to move the arm to a position where the hook is 18 inches above the shoulder.
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

        # ! This sets a new angle target for the shoulders only if the shoulders are inside the angle bounds of 30 degrees and 180 degrees.
        # ! The arms start at about 30 degrees from straight down, and they should never need to point past straight up (180 degrees).
        # ! This does not allow the arm to be moved back once it has moved too far, but that is something it should do at some point.
        if abs(switchDict["shoulderValue"]) > 0.1:
            if self.shoulderTargetAngle + switchDict["shoulderValue"] > 180 or self.shoulderTargetAngle + switchDict["shoulderValue"] < 30:
                pass
            else:
                self.shoulderTargetAngle += switchDict["shoulderValue"]

        # ! This is where the shoulders actually move.
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
    


    # ! This method takes the deadzone values from the config and compares them to the values it recieves.
    # ! If the values are smaller than the deadzone then the function outputs 0 in place of the input value.
    def evaluateDeadzones(self, inputs):
        '''This method takes in a list consisting of x input, y input, z input, arm input, and winch input.
        The magnitude of the units has to be less than 1.
        Returns the list of inputs with zero in place of values less than their respective deadzones.'''
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
    


    # ! This stops everything on the robot except the climber. I don't know why it doesn't stop the climber.
    def nonEmergencyStop(self):
        ''' Exactly as it says, stops all of the functions of the robot '''
        self.driveTrain.stationary()
        self.tower.towerCoast()
        self.intake.carWashOff()
        self.tower.coastShooter()

if __name__ == "__main__":
    wpilib.run(MyRobot)
