# All imports at the top of the file, including the other files we made in the repository.
import wpilib
import navx
import autonomous
import driveTrain


config = {
  "SwerveModules": {
    "frontLeft": {
      "motor_ID_1": 1,
      "motor_ID_2": 2,
      "encoder_ID": 3,
      "encoderOffset": 0.0
    },
    "frontRight": {
      "motor_ID_1": 4,
      "motor_ID_2": 5,
      "encoder_ID": 6,
      "encoderOffset": 0.0
    },
    "rearLeft": {
      "motor_ID_1": 7,
      "motor_ID_2": 8,
      "encoder_ID": 9,
      "encoderOffset": 0.0
    },
    "rearRight": {
      "motor_ID_1": 10,
      "motor_ID_2": 11,
      "encoder_ID": 12,
      "encoderOffset": 0.0
    }
  },
  "RobotDimensions": { # fix these
    "trackWidth": 36,
    "wheelBase": 48,
    "wheelDiameter": 4
  },
  "RobotDefaultSettings": {
    "fieldOrient": False,
    "easterEgg": "e"
  },
  "driverStation": {
    "joystickDeadZones": {
      "xDeadZone": 0.1,
      "yDeadZone": 0.1,
      "zDeadZone": 0.1
    }
  },
  }

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.driveTrain = driveTrain.driveTrain(config)
        self.navx = navx.AHRS.create_spi()
    def autonomousInit(self):
        autoPlanName = "default"
        self.autonomousController = autonomous.autonomous(autoPlanName)
        self.navx.reset()
        self.navx.resetDisplacement()
    def autonomousPeriodic(self):
        x, y, z, auxiliary = self.autonomousController.periodic(self.navx.getDisplacementX() * 39.37008, self.navx.getDisplacementY() * 39.37008) # need to eventually add support for auxiliary systems
        if x != 0 or y != 0 or z != 0:
            self.driveTrain.manualMove(x, y, z, -1*self.navx.getAngle() + 90)
        else:
            self.driveTrain.stationary()
    def teleopInit(self):
        self.driverInput = wpilib.Joystick(0)
        self.enabledToZero = False
    def teleopPeriodic(self):
        if not self.enabledToZero:
            tolerance = 0.25
            self.driveTrain.enableToZeros()
            driveData = self.driveTrain.refreshValues()
            for module in driveData:
                motorPosition = module[0]
                if motorPosition + tolerance > 0 and motorPosition - tolerance < 0:
                    self.enabledToZero = True
        else:
            switches = self.checkSwitches()
            switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones(switches["driverX"], switches["driverY"], switches["driverZ"])
            if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
                self.driveTrain.manualMove(switches["driverX"], switches["driverY"], switches["driverZ"], switches["navxAngle"])
            else:
                self.driveTrain.stationary()
            
    def testInit(self):
        pass
    def testPeriodic(self):
        pass
    def disabledInit(self):
        pass
    def disabledPeriodic(self):
        pass
    def checkSwitches(self):
        switchDict = {
            "driverX": 0.0,
            "driverY": 0.0,
            "driverZ": 0.0,
            "navxAngle": -1*self.navx.getAngle() + 90,
            "calibrateDriveTrainEncoders": False
        }
        switchDict["driverX"] = self.driverInput.getX()
        switchDict["driverY"] = self.driverInput.getY()
        switchDict["driverZ"] = self.driverInput.getZ()
        return switchDict
    


if __name__ == "__main__":
    wpilib.run(MyRobot)