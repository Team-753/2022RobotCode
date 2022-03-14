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
            "swerveAfterburners": False,
            "swapFieldOrient": False,
            "intakeUp": False,
            "intakeDown": False,
            "revShooter": False, # revs the shooter
            "ballIndexerIn": False, # runs and compliants and feeder (if no ball is at the top)
            "ballSystemOut": False, # reverses the feeder, compliants, and intake
            "releasePeterHooks": False,
            "tightenPeterHooks": False,
            "moveArms": 0.0,
            "moveWinches": 0.0,
            "intakeOn": False,
            "resetDriveTrainEncoders": False,
            "leftWinchIn": False,
            "rightWinchIn": False,
            "leftWinchOut": False,
            "rightWinchOut": False,
            "shoulderClockwise": False,
            "shoulderCounterClockwise": False,
            "armHome": False,
            "armStraightUp": False,
            "armToHooks": False,
            "winchesIn": False,
            "winchesOut": False,
            "shoulderValue": 0.0,
            "revShooterClose": False
        }
        
        switches["driverX"] = -self.driverInput.getLeftX()
        switches["driverY"] = self.driverInput.getLeftY()
        switches["driverZ"] = self.driverInput.getRightTriggerAxis() - self.driverInput.getLeftTriggerAxis()
        #switches["swerveAfterburners"] = self.driverInput.getLeftBumper()
        if self.driverInput.getBackButtonReleased() and not self.climbCheckOne:
            switches["swapFieldOrient"] = True
        leftAxis = self.auxiliaryInput.getLeftY()
        if leftAxis > 0.5:
            switches["shoulderCounterClockwise"] = True
        elif leftAxis > 0.5:
            switches["shoulderClockwise"] = True
        dPadStateDriver = self.driverInput.getPOV()
        if dPadStateDriver == 90:
            #switches["winchIn"] = True
            pass
        elif dPadStateDriver == 270:
            #switches["winchOut"] = True
            pass
        elif dPadStateDriver == 0:
            switches["intakeUp"] = True
        elif dPadStateDriver == 180:
            switches["intakeDown"] = True
        dPadStateAux = self.auxiliaryInput.getPOV()
        if dPadStateAux == 0:
            #switches["armStraightUp"] = True
            switches["winchesOut"] = True
        elif dPadStateAux == 180:
            #switches["armToHooks"] = True
            switches["winchesIn"] = True
        if self.auxiliaryInput.getBackButton():
            switches["armHome"] = True
        if self.auxiliaryInput.getRightTriggerAxis() > self.config["driverStation"]["flywheelTriggerThreshold"]:
            switches["revShooter"] = True
        elif self.auxiliaryInput.getLeftTriggerAxis() > self.config["driverStation"]["flywheelTriggerThreshold"]:
            switches["revShooterClose"] = True
        if self.auxiliaryInput.getLeftBumper() or self.driverInput.getLeftBumper():
            switches["intakeOn"] = True
        switches["ballSystemOut"] = self.auxiliaryInput.getBButton()
        switches["ballIndexerIn"] = self.auxiliaryInput.getRightBumper() or self.driverInput.getRightBumper()  #will do checks on this later ie: if flywheel is at sufficient rpm and such
        switches["releasePeterHooks"] = self.auxiliaryInput.getYButtonReleased()
        switches["tightenPeterHooks"] = self.auxiliaryInput.getAButtonReleased()
        #switches["testVolts"] = self.auxiliaryInput.getLeftTriggerAxis() * 12
        if self.auxiliaryInput.getStartButton():
            if self.auxiliaryInput.getPOV() == 90:
                switches["rightWinchOut"] = True
            if self.auxiliaryInput.getPOV() == 270:
                switches["leftWinchOut"] = True
        if dPadStateAux == 90 and not self.auxiliaryInput.getStartButton():
            switches["rightWinchIn"] = True
        if dPadStateAux == 270 and not self.auxiliaryInput.getStartButton():
            switches["leftWinchIn"] = True
        switches["shoulderValue"] = -self.auxiliaryInput.getLeftY()
        
        

        return switches