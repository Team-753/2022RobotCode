# All imports at the top of the file, including the other files we made in the repository.
import wpilib
import navx
import driveTrain
import autonomous
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
		self.useXboxController = True
		self.integralDefault = 0.0025
		self.proportionalDefault = 0.006
		self.derivativeDefault = 0
		wpilib.SmartDashboard.putNumber("Integral", self.integralDefault)
		wpilib.SmartDashboard.putNumber("Proportional", self.proportionalDefault)
		wpilib.SmartDashboard.putNumber("Derivative", self.derivativeDefault)
			# self.camera = wpilib.CameraServer()
			# self.camera.launch()
			# self.piston = wpilib.DoubleSolenoid(reverseChannel=self.config["PCM"]["pistonForward_ID"], forwardChannel=self.config["PCM"]["pistonReverse_ID"], moduleNumber=self.config["PCM"]["PCM_ID"])

	def autonomousInit(self):
		self.auto = autonomous.Autonomous("default", self.navx)
  
	def disabledPeriodic(self):
		vals = self.driveTrain.refreshValues()
		wpilib.SmartDashboard.putNumber("FLA", vals[0][1])
		wpilib.SmartDashboard.putNumber("FRA", vals[1][1])
		wpilib.SmartDashboard.putNumber("RLA", vals[2][1])
		wpilib.SmartDashboard.putNumber("RRA", vals[3][1])
		integral = wpilib.SmartDashboard.getNumber("Integral", self.integralDefault)
		proportion = wpilib.SmartDashboard.getNumber("Proportional", self.proportionalDefault)
		derivative = wpilib.SmartDashboard.getNumber("Derivative", self.derivativeDefault)
		if integral != self.integralDefault:
			self.integralDefault = integral
			self.driveTrain.swerveModules["frontLeft"].turnController.setI(self.integralDefault)
		if proportion != self.proportionalDefault:
			self.proportionalDefault = proportion
			self.driveTrain.swerveModules["frontLeft"].turnController.setP(self.proportionalDefault)
		if derivative != self.derivativeDefault:
			self.derivativeDefault = derivative
			self.driveTrain.swerveModules["frontLeft"].turnController.setD(self.derivativeDefault)


	def autonomousPeriodic(self):
		pose = self.driveTrain.getFieldPosition()
		x, y, z, auxiliary = self.auto.periodic(pose)
		print(x, y, z, auxiliary)
		if x != 0 or y != 0 or z != 0:
			self.driveTrain.move(x, y, z)
		else:
			self.driveTrain.stationary()

	def teleopInit(self):
		self.driverInput = wpilib.Joystick(0)
		self.xboxInput = wpilib.XboxController(1)
		self.navx.reset() # remove in production code
		self.driveTrain.resetOdometry()

	def teleopPeriodic(self):
		switches = self.checkSwitches()
		self.driveTrain.updateOdometry()
		position = self.driveTrain.getFieldPosition()
		wpilib.SmartDashboard.putNumber("X Position", position[0])
		wpilib.SmartDashboard.putNumber("Y Position", position[1])
		wpilib.SmartDashboard.putNumber("Z Rotation", position[2])
		# switches["driverX"], switches["driverY"], switches["driverZ"] = self.evaluateDeadzones([switches["driverX"], switches["driverY"], switches["driverZ"]])
		if switches["driverX"] != 0 or switches["driverY"] != 0 or switches["driverZ"] != 0:
			self.driveTrain.move(switches["driverX"], switches["driverY"], switches["driverZ"])
		else:
			#self.driveTrain.move(0,0,0)
			self.driveTrain.stationary()
		if switches["calibrateDriveTrainEncoders"]:
			self.driveTrain.reInitiateMotorEncoders()

		integral = wpilib.SmartDashboard.getNumber("Integral", self.integralDefault)
		proportion = wpilib.SmartDashboard.getNumber("Proportional", self.proportionalDefault)
		derivative = wpilib.SmartDashboard.getNumber("Derivative", self.derivativeDefault)
		if integral != self.integralDefault:
			self.integralDefault = integral
			self.driveTrain.swerveModules["frontLeft"].turnController.setI(self.integralDefault)
		if proportion != self.proportionalDefault:
			self.proportionalDefault = proportion
			self.driveTrain.swerveModules["frontLeft"].turnController.setP(self.proportionalDefault)
		if derivative != self.derivativeDefault:
			self.derivativeDefault = derivative
			self.driveTrain.swerveModules["frontLeft"].turnController.setD(self.derivativeDefault)

	
	def checkSwitches(self):
		switchDict = {
			"driverX": 0.0,
			"driverY": 0.0,
			"driverZ": 0.0,
			"navxAngle": -1*self.navx.getAngle() + 90,
			"calibrateDriveTrainEncoders": False
		}
		x = self.driverInput.getX()
		y = -self.driverInput.getY()
		z = self.driverInput.getZ()
		x, y, z = self.evaluateDeadzones((x, y, z))
		if x == 0 and y == 0 and z == 0 and self.useXboxController:
			x = self.xboxInput.getLeftX()
			y = -self.xboxInput.getLeftY()
			z = -self.xboxInput.getRightX()
			x, y, z = self.evaluateDeadzones((x, y, z))
		switchDict["driverX"], switchDict["driverY"], switchDict["driverZ"] = x, y, z
		switchDict["calibrateDriveTrainEncoders"] = self.driverInput.getRawButtonReleased(11)
		#print(f)
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