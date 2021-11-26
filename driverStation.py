import wpilib

class driverStation:
    def __init__(self, config):
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
        switchList = {
            "driverX": 0.0,
            "driverY": 0.0,
            "driverZ": 0.0,
            "swapFieldOrient": False
        }
        if self.driverInputType != "disconnected":
            if self.driverInputType == "XboxController":
                switchList["driverX"] = self.driverInput.Axis.kLeftX
                switchList["driverY"] = self.driverInput.Axis.kLeftY
                switchList["driverZ"] = self.driverInput.Axis.kRightX # NOTE: Use this for turning
                '''
                NOTE: Alternatively use this which uses the triggers for turning which may seem weird but could be more accurate:
                switchList["driverZ"] = self.driverInput.Axis.kRightTrigger - self.driverInput.Axis.kLeftTrigger
                '''
                
            else: # Joystick
                switchList["driverX"] = self.driverInput.getX()
                switchList["driverY"] = self.driverInput.getY()
                switchList["driverZ"] = self.driverInput.getZ()
            # after this point is auxilary code
            switchList["swapFieldOrient"] == self.driverInput.getBackButtonReleased()
        else:
            self.checkDriverStationInputs()
        return switchList