import wpilib

class driverStation:
    def __init__(self, config: dict):
        self.config = config
        self.driverInput = wpilib.XboxController(0)
        self.auxiliaryInput = wpilib.XboxController(1)
        self.climbCheckOne = False
        self.climbCheckTwo = False
        self.climbCheckThree = False
        self.climbModeActivated = False
        self.manualClimbing = True # for now...
        self.peterHookCheckRelease = 0
        self.peterHookCheckTighten = 0
    
    def checkSwitches(self):
        switches = {
            "driverX": 0.0,
            "driverY": 0.0,
            "driverZ": 0.0,
            "swapFieldOrient": False,
            "intakeUp": False,
            "intakeDown": False,
            "revShooter": False, # revs the shooter
            "ballIndexerIn": False, # runs and compliants and feeder (if no ball is at the top)
            "ballSystemOut": False, # reverses the feeder, compliants, and intake
            "toggleClimbMode": False,
            "releasePeterHooks": False,
            "tightenPeterHooks": False,
            "moveArms": 0.0,
            "moveWinches": 0.0,
            "intakeOn": False,
            "intakeOff": False,
            "resetDriveTrainEncoders": False,
            "macros": {}
        }
        
        
        switches["toggleClimbMode"] = self.checkClimbingModeToggle()
        if switches["toggleClimbMode"]:
            if self.climbModeActivated:
                self.climbModeActivated = False
            elif not self.climbModeActivated:
                self.climbModeActivated = True
        
        if not self.climbModeActivated: # normal driving mode
            switches["driverX"] = self.driverInput.getLeftX()
            switches["driverY"] = self.driverInput.getLeftY()
            switches["driverZ"] = self.driverInput.getRightX()
            if self.driverInput.getBackButtonReleased() and not self.climbCheckOne:
                switches["swapFieldOrient"] = True
            dPadState = self.auxiliaryInput.getPOV()
            if dPadState == 90:
                switches["intakeUp"] = True
            elif dPadState == 270:
                switches["intakeDown"] = True
            elif dPadState == 0:
                switches["intakeOn"] = True
            elif dPadState == 180:
                dPadState["intakeOff"] = True
            if self.auxiliaryInput.getRightTriggerAxis() > self.config["driverStation"]["flywheelTriggerThreshold"]:
                switches["revShooter"] = True
            switches["ballSystemOut"] = self.auxiliaryInput.getBButtonPressed()
            switches["ballIndexerIn"] = self.auxiliaryInput.getRightBumperPressed()  #will do checks on this later ie: if flywheel is at sufficient rpm and such 
        else:
            if self.manualClimbing:
                switches["releasePeterHooks"], switches["tightenPeterHooks"] = self.peterHooks()
                switches["moveArms"] = self.auxiliaryInput.getLeftY()
                switches["moveWinches"] = self.auxiliaryInput.getRightY()

        return switches

    def peterHooks(self):
        if self.peterHookCheckRelease > 0:
            self.peterHookCheckRelease += 1
        if self.peterHookCheckRelease > 50:
            self.peterHookCheckRelease = 0
        if self.peterHookCheckTighten > 0:
            self.peterHookCheckTighten += 1
        if self.peterHookCheckTighten > 50:
            self.peterHookCheckTighten = 0
        if self.auxiliaryInput.getYButtonReleased():
            if self.peterHookCheckRelease == 0:
                self.peterHookCheckRelease = 1
            elif self.peterHookCheckRelease > 0:
                self.peterHookCheckRelease = 0
                releasePeterHooks = True
        if self.auxiliaryInput.getAButtonReleased():
            if self.peterHookCheckTighten == 0:
                self.peterHookCheckTighten = 1
            elif self.peterHookCheckTighten > 0:
                self.peterHookCheckTighten = 0
                tightenPeterHooks = True
        return(releasePeterHooks, tightenPeterHooks)
    
    def checkClimbingModeToggle(self):
        if self.auxiliaryInput.getBackButtonPressed() and self.auxiliaryInput.getStartButtonPressed():
            self.climbCheckOne = True
        if self.auxiliaryInput.getBackButtonReleased() and self.climbCheckOne:
            self.climbCheckTwo = True
        if self.auxiliaryInput.getStartButtonReleased() and self.climbCheckOne:
            self.climbCheckThree = True
        if self.climbCheckOne and self.climbCheckTwo and self.climbCheckThree:
            ''' So what is going on here you may ask? Essentially this is the toggle for the climbing mode activation.
                to 'toggle' climbing, you must press both the back and start buttons at the same time and then release them at any time.
                These checks first insure that they are being pressed at the same time and then individually check if they have been released so it
                only sends the toggle command once. You do not want to toggle multiple times over for something like this.'''
            self.climbCheckOne = False
            self.climbCheckTwo = False
            self.climbCheckThree = False
            return(True)