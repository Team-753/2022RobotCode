# All imports at the top of the file, including the other files we made in the repository.
import wpilib
import navx
import driveTrain
import autonomous
import math
import time
import os
import json


class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		folderPath = os.path.dirname(os.path.abspath(__file__))
		filePath = os.path.join(folderPath, 'config.json')
		with open (filePath, "r") as f1:
			self.config = json.load(f1)
		self.navx = navx.AHRS(wpilib._wpilib.I2C.Port.kOnboard, update_rate_hz=100)
		self.driveTrain = driveTrain.driveTrain(self.config, self.navx)
			# self.camera = wpilib.CameraServer()
			# self.camera.launch()
			# self.piston = wpilib.DoubleSolenoid(reverseChannel=self.config["PCM"]["pistonForward_ID"], forwardChannel=self.config["PCM"]["pistonReverse_ID"], moduleNumber=self.config["PCM"]["PCM_ID"])

	def autonomousInit(self):
		self.auto = autonomous.Autonomous("default", self.navx)

	def autonomousPeriodic(self):
		self.driveTrain.updateOdometry()
		pose = self.driveTrain.getFieldPosition()
		x, y, z, auxiliary = self.auto.periodic(pose)
		if x != 0 or y != 0 or z != 0:
			self.driveTrain.move(x, y, z)
		else:
			self.driveTrain.stationary()

	def teleopInit(self):
		self.driverInput = wpilib.Joystick(0)
		self.navx.reset() # remove in production code

	def teleopPeriodic(self):
		switches = self.checkSwitches()
		switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones([switches["driverX"], switches["driverY"], switches["driverZ"]])
		if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
			self.driveTrain.move(switches["driverX"], switches["driverY"], switches["driverZ"])
		else:
			self.driveTrain.stationary()
	
	def checkSwitches(self):
		switchDict = {
			"driverX": 0.0,
			"driverY": 0.0,
			"driverZ": 0.0,
			"navxAngle": -1*self.navx.getAngle() + 90,
			"calibrateDriveTrainEncoders": False
		}
		switchDict["driverX"] = self.driverInput.getX()
		switchDict["driverY"] = -self.driverInput.getY()
		switchDict["driverZ"] = self.driverInput.getZ()
		return switchDict
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
		return adjustedInputs[0], adjustedInputs[1], adjustedInputs[2]

if __name__ == "__main__":
	wpilib.run(MyRobot)