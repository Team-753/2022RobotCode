import wpilib

class driverStation:
    def __init__(self, config: dict):
        self.config = config
        self.driverInput = wpilib.XboxController(0)
    
    def checkSwitches(self):
        switchDict = {
            "driverX": 0.0,
            "driverY": 0.0,
            "driverZ": 0.0,
            "swapFieldOrient": False,
            "intakeUp": False,
            "intakeDown": False,
            "IntakeOn": False,
            "IntakeOff": False
        }
        switchDict["driverX"] = self.driverInput.getLeftX()
        switchDict["driverY"] = self.driverInput.getLeftY()
        switchDict["driverZ"] = self.driverInput.getRightX()
        switchDict["intakeUp"] = self.driverInput
        return switchDict
