# All imports at the top of the file, including the other files we made in the repository.
import wpilib
import navx
import driveTrain
import autonomous
import math
import time


config = {
	"PCM": {
		"pistonForward_ID": 1,
		"pistonReverse_ID": 0
	},
	"SwerveModules": {
		"frontLeft": {
			"motor_ID_1": 1,
			"motor_ID_2": 3,
			"encoder_ID": 2,
			"encoderOffset": 16.787109
		},
		"frontRight": {
			"motor_ID_1": 4,
			"motor_ID_2": 6,
			"encoder_ID": 5,
			"encoderOffset": -109.511719
		},
		"rearLeft": {
			"motor_ID_1": 7,
			"motor_ID_2": 9,
			"encoder_ID": 8,
			"encoderOffset": -103.271484
		},
		"rearRight": {
			"motor_ID_1": 10,
			"motor_ID_2": 12,
			"encoder_ID": 11,
			"encoderOffset": -124.101563
		}
	},
	"RobotDimensions": { # fix these
		"trackWidth": 26.5,
		"wheelBase": 26.5,
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
			"zDeadZone": 0.2
		}
	},
}

class MyRobot(wpilib.TimedRobot):
		def robotInit(self):
				self.driveTrain = driveTrain.driveTrain(config)
				self.navx = navx.AHRS.create_spi(update_rate_hz=100)

				self.piston = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, config["PCM"]["pistonForward_ID"], config["PCM"]["pistonReverse_ID"])

		def autonomousInit(self):
				autoPlanName = "default"
				self.autonomousController = autonomous(autoPlanName)
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
				self.navx.reset()
		def teleopPeriodic(self):
				switches = self.checkSwitches()
				switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones(switches["driverX"], switches["driverY"], switches["driverZ"])
				if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
						self.driveTrain.manualMove(switches["driverX"], switches["driverY"], switches["driverZ"], switches["navxAngle"])
				else:
						self.driveTrain.stationary()
				vals = self.driveTrain.refreshValues()
				wpilib.SmartDashboard.putNumber("FLA", vals[0][1])
				wpilib.SmartDashboard.putNumber("FRA", vals[1][1])
				wpilib.SmartDashboard.putNumber("RLA", vals[2][1])
				wpilib.SmartDashboard.putNumber("RRA", vals[3][1])
				wpilib.SmartDashboard.putNumber("navx Angle:", self.navx.getAngle())
				wpilib.SmartDashboard.putNumber("navx X Displacement:", self.navx.getDisplacementX())
				wpilib.SmartDashboard.putNumber("navx Z Displacement:", self.navx.getDisplacementZ())

				if self.driverInput.getRawButton(10):
					self.piston.set(1)
				elif self.driverInput.getRawButton(11):
					self.piston.set(2)
				else:
					self.piston.set(0)
						
		def testInit(self):
				pass
		def testPeriodic(self):
				
				pass

		def disabledInit(self):
				pass
		def disabledPeriodic(self):
				vals = self.driveTrain.refreshValues()
				wpilib.SmartDashboard.putNumber("FLA", vals[0][1])
				wpilib.SmartDashboard.putNumber("FRA", vals[1][1])
				wpilib.SmartDashboard.putNumber("RLA", vals[2][1])
				wpilib.SmartDashboard.putNumber("RRA", vals[3][1])
				wpilib.SmartDashboard.putNumber("FL", vals[0][0])
				wpilib.SmartDashboard.putNumber("FR", vals[1][0])
				wpilib.SmartDashboard.putNumber("RL", vals[2][0])
				wpilib.SmartDashboard.putNumber("RR", vals[3][0])
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
		def evaluateDeadzones(self, x: float, y: float, z: float):
			if not (x > config["driverStation"]["joystickDeadZones"]["xDeadZone"] or x < -config["driverStation"]["joystickDeadZones"]["xDeadZone"]):
					x = 0
			if not (y > config["driverStation"]["joystickDeadZones"]["yDeadZone"] or y < -config["driverStation"]["joystickDeadZones"]["yDeadZone"]):
					y = 0
			if not (z > config["driverStation"]["joystickDeadZones"]["zDeadZone"] or z < -config["driverStation"]["joystickDeadZones"]["zDeadZone"]):
					z = 0
			
			return x, y, z

if __name__ == "__main__":
		wpilib.run(MyRobot)