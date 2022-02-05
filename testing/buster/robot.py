# All imports at the top of the file, including the other files we made in the repository.
from hal import I2CPort
import wpilib
import navx
import driveTrain
from autonomous import Autonomous
import math
import time


config = {
	"PCM": {
		"pistonForward_ID": 1,
		"pistonReverse_ID": 0,
		"PCM_ID": 61
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
		"fieldOrient": True,
		"easterEgg": "e"
	},
	"driverStation": {
		"joystickDeadZones": {
			"xDeadZone": 0.1,
			"yDeadZone": 0.1,
			"zDeadZone": 0.15
		}
	},
}

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
			self.driveTrain = driveTrain.driveTrain(config)
			self.navx = navx.AHRS(wpilib._wpilib.I2C.Port.kOnboard, update_rate_hz=100)
			#self.camera = wpilib.CameraServer()
			# self.camera.launch()

			#self.piston = wpilib.DoubleSolenoid(reverseChannel=config["PCM"]["pistonForward_ID"], forwardChannel=config["PCM"]["pistonReverse_ID"], moduleNumber=config["PCM"]["PCM_ID"])


	def autonomousInit(self):
		pass
		'''self.navx.reset()
		self.navx.resetDisplacement()
		autoPlanName = "default"
		self.autonomousController = Autonomous(autoPlanName)
		self.navx.setAngleAdjustment(self.autonomousController.initialAngle)'''
	def autonomousPeriodic(self):
		pass
		'''
		x, y, z, auxiliary = self.autonomousController.periodic(self.navx.getDisplacementX() * 39.37008, self.navx.getDisplacementY() * 39.37008) # need to eventually add support for auxiliary systems
		if x != 0 or y != 0 or z != 0:
				self.driveTrain.manualMove(x, y, z, -1*self.navx.getAngle() + 90)
		else:
				self.driveTrain.stationary()
		'''
	def teleopInit(self):
		self.driverInput = wpilib.Joystick(0)
		self.enabledToZero = False
		self.navx.reset()
		self.navx.setAngleAdjustment(30)
		self.navx.updateDisplacement(3, -2, 100, False)
	def diagnostics(self):
		'''wpilib.SmartDashboard.putNumber("navx Angle:", self.navx.getAngle())
		wpilib.SmartDashboard.putNumber("navx X Displacement:", self.navx.getDisplacementX())
		wpilib.SmartDashboard.putNumber("navx Y Displacement:", self.navx.getDisplacementY())
		wpilib.SmartDashboard.putNumber("navx Z Displacement:", self.navx.getDisplacementZ())'''
	def teleopPeriodic(self):
		switches = self.checkSwitches()
		switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones([switches["driverX"], switches["driverY"], switches["driverZ"]])
		if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
				self.driveTrain.move(switches["driverX"], switches["driverY"], switches["driverZ"], switches["navxAngle"])
		else:
				self.driveTrain.stationary()
		self.diagnostics()
				
	def testInit(self):
		pass
	def testPeriodic(self):
		pass
	def disabledInit(self):
		pass
	def disabledPeriodic(self):
		self.diagnostics()
	def checkSwitches(self):
		switchDict = {
				"driverX": 0.0,
				"driverY": 0.0,
				"driverZ": 0.0,
				"navxAngle": -1*self.navx.getAngle() + 90,
				"calibrateDriveTrainEncoders": False
		}
		switchDict["driverX"] = self.driverInput.getX()*0.5
		switchDict["driverY"] = self.driverInput.getY()*0.5
		switchDict["driverZ"] = self.driverInput.getZ()*0.5
		return switchDict
	def evaluateDeadzones(self, inputs):
		adjustedInputs = []
		for idx, input in enumerate(inputs):
			threshold = config["driverStation"]["joystickDeadZones"][(list(config["driverStation"]["joystickDeadZones"])[idx])]
			if abs(input) > threshold: 
				adjustedValue = (abs(input) - threshold) / (1 - threshold)
				if input < 0 and adjustedValue != 0:
					adjustedValue = -adjustedValue
			else:
				adjustedValue = 0
			adjustedInputs.append(adjustedValue)
		# wpilib.SmartDashboard.putNumberArray("Joystick Adjusted Vals", adjustedInputs)
		return adjustedInputs[0], adjustedInputs[1], adjustedInputs[2]

if __name__ == "__main__":
	wpilib.run(MyRobot)