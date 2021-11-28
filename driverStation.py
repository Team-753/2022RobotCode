import wpilib
from robot import navx

class driverStation:
    def __init__(self, config: dict):
        self.driverStationUtil = wpilib.DriverStation()
        self.config = config
        
        self.checkDriverStationInputs()
        
    
    def checkDriverStationInputs(self):
        driverInputName = self.driverStationUtil.getJoystickName(0) # Driverstation input is on port 0
        auxilaryInputName = self.driverStationUtil.getJoystickName(1) # Auxilary input is on port 1
        if driverInputName == "Logitech Extreme 3D": # This string is correct given we are using this joystick
            self.driverInputType = "Joystick"
            self.driverInput = wpilib.Joystick(0)
        elif driverInputName == "Controller (Xbox One For Windows)": # This string may be wrong and needs to be updated once I can see what the correct output is.
            self.driverInputType = "XboxController"
            self.driverInput = wpilib.XboxController(0)
        else:
            self.driverStationUtil.reportWarning("Driver input is either unplugged or not set to USB0")
            self.driverInputType = "disconnected"
        if auxilaryInputName != "Controller (Xbox One For Windows)":
            self.driverStationUtil.reportWarning("Auxilary input is either unplugged or not set to USB1")
        self.auxilaryInput = wpilib.XboxController(1)
    
    def checkSwitches(self):
        switchDict = {
            "driverX": 0.0,
            "driverY": 0.0,
            "driverZ": 0.0,
            "swapFieldOrient": False,
            "playEasterEgg": False,
            "fieldOrient": wpilib.SmartDashboard.getBoolean("fieldOrient", True),
            "navxAngle": -1*navx.getAngle() + 90,
            "zeroDriveTrainEncoders": False
        }
        if self.driverInputType != "disconnected":
            if self.driverInputType == "XboxController":
                switchDict["driverX"] = self.driverInput.Axis.kLeftX
                switchDict["driverY"] = self.driverInput.Axis.kLeftY
                switchDict["driverZ"] = self.driverInput.Axis.kRightX # NOTE: Use this for turning
                '''
                NOTE: Alternatively use this which uses the triggers for turning which may seem weird but could be more accurate:
                switchDict["driverZ"] = self.driverInput.Axis.kRightTrigger - self.driverInput.Axis.kLeftTrigger
                '''
                
            else: # Joystick
                switchDict["driverX"] = self.driverInput.getX()
                switchDict["driverY"] = self.driverInput.getY()
                switchDict["driverZ"] = self.driverInput.getZ()
            # after this point is auxilary code
            switchDict["swapFieldOrient"] = self.auxilaryInput.getStartButtonReleased()
            switchDict["playEasterEgg"] = self.auxilaryInput.getBButtonReleased()
            switchDict["zeroDriveTrainEncoders"] = self.auxilaryInput.getBackButtonReleased()
        else:
            self.checkDriverStationInputs()
        return switchDict