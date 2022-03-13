import math
import wpilib
import ctre
import rev
import wpimath.controller

class Climber:
    def __init__(self, config: dict):
        self.config = config
        
        self.leftShoulder = Shoulder(self.config["Climber"]["leftShoulder"]["ID"], "leftShoulder", self.config)
        self.rightShoulder = Shoulder(self.config["Climber"]["rightShoulder"]["ID"], "rightShoulder", self.config)

        self.leftWinch = Winch(self.config["Climber"]["leftWinch"]["ID"], self.config["Climber"]["leftWinch"]["Name"], self.config)
        self.rightWinch = Winch(self.config["Climber"]["rightWinch"]["ID"], self.config["Climber"]["rightWinch"]["Name"], self.config)

        self.leftArm = Arm(self.leftWinch, self.leftShoulder, self.config)
        self.rightArm = Arm(self.rightWinch, self.rightShoulder, self.config)
        
        self.leftHook = wpilib.DoubleSolenoid(self.config["PCM"], wpilib.PneumaticsModuleType.CTREPCM, forwardChannel = self.config["Climber"]["leftHook_PCM_ID_Forward"], reverseChannel = self.config["Climber"]["leftHook_PCM_ID_Reverse"])
        self.rightHook = wpilib.DoubleSolenoid(self.config["PCM"], wpilib.PneumaticsModuleType.CTREPCM, forwardChannel = self.config["Climber"]["rightHook_PCM_ID_Forward"], reverseChannel = self.config["Climber"]["rightHook_PCM_ID_Reverse"])

    def moveWinches(self, speed):
        self.leftArm.moveWinch(speed)
        self.rightArm.moveWinch(speed)
    
    def moveShoulders(self, speed):
        self.leftArm.moveShoulder(speed)
        self.rightArm.moveShoulder(speed)
        
    def setArms(self, x, y):
        self.leftArm.moveTo(x, y)
        self.rightArm.moveTo(x, y)

    def coastArms(self):
        self.leftArm.coast()
        self.rightArm.coast()
    
    def brakeArms(self):
        self.leftArm.brake()
        self.rightArm.brake()
        
    def armsStraightUp(self):
        self.leftArm.moveTo()
    
    def RTH(self):
        self.leftArm.setShoulderAngle(30)
        self.rightArm.setShoulderAngle(30)
        self.leftArm.setWinchPosition(0)
        self.rightArm.setWinchPosition(0)
    
    def getArmPositions(self):
        wpilib.SmartDashboard.putNumber("Left Arm X", self.leftArm.getArmInverseKinematics(self.leftArm.winch.getWinchLength(), self.leftArm.shoulder.getAngle())[0])
        wpilib.SmartDashboard.putNumber("Right Arm X", self.rightArm.getArmInverseKinematics(self.rightArm.winch.getWinchLength(), self.rightArm.shoulder.getAngle())[0])
        wpilib.SmartDashboard.putNumber("Left Arm Y", self.leftArm.getArmInverseKinematics(self.leftArm.winch.getWinchLength(), self.leftArm.shoulder.getAngle())[1])
        wpilib.SmartDashboard.putNumber("Right Arm Y", self.rightArm.getArmInverseKinematics(self.rightArm.winch.getWinchLength(), self.rightArm.shoulder.getAngle())[1])
        wpilib.SmartDashboard.putNumber("Left Arm Winch Length", self.leftArm.winch.getWinchLength())
        wpilib.SmartDashboard.putNumber("Left Arm Shoulder Angle", self.leftArm.shoulder.getAngle())
        wpilib.SmartDashboard.putNumber("Right Arm Winch Length", self.rightArm.winch.getWinchLength())
        wpilib.SmartDashboard.putNumber("Right Arm Shoulder Angle", self.rightArm.shoulder.getAngle())
        
    def engageHooks(self):
        self.leftHook.set(wpilib.DoubleSolenoid.Value.kForward)
        self.rightHook.set(wpilib.DoubleSolenoid.Value.kForward)

    def disengageHooks(self):
        self.leftHook.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.rightHook.set(wpilib.DoubleSolenoid.Value.kReverse)

class Arm:
    def __init__(self, winch, shoulder, config) -> None:
        self.winch = winch
        self.shoulder = shoulder
        self.config = config
        self.armLength = self.config["Climber"]["Arm_Dimensions"]["Arm"]
        self.forearmLength = math.hypot(self.config["Climber"]["Arm_Dimensions"]["Forearm1"], self.config["Climber"]["Arm_Dimensions"]["Forearm2"])
        self.pulleyJointDifference = self.config["Climber"]["Arm_Dimensions"]["PulleyJointDifference"]

        self.shoulderPID = wpimath.controller.PIDController(self.config["Climber"]["shoulderPID"]["kP"], self.config["Climber"]["shoulderPID"]["kI"], self.config["Climber"]["shoulderPID"]["kD"])
        self.winchPID = wpimath.controller.PIDController(self.config["Climber"]["winchPID"]["kP"], self.config["Climber"]["winchPID"]["kI"], self.config["Climber"]["winchPID"]["kD"])

    def getArmInverseKinematics(self, x, y):
        '''This returns the angle of the shoulder and the length of strap let out of the winch based on the desired x and y position of the arm hook.'''
        theta = (math.pi/2) + math.acos(x/(math.hypot(x,y))) - math.acos(((x**2) + (y**2) + (self.armLength**2) - (self.forearmLength**2))/(2*self.armLength*math.hypot(x,y)))
        theta = theta*180/math.pi
        winchLength = math.hypot(x,(y - self.pulleyJointDifference))
        return(winchLength, theta)
    
    '''
    def getArmKinematics(self, winchLength, theta1):
        #* This math might be wrong but I don't know how to test that yet.
        theta2 = math.acos(((L5**2) + (self.armLength**2) - (self.forearmLength**2))/(2*L5*self.armLength))
        L5 = ((2*self.pulleyJointDifference*math.cos(math.pi - theta1 - theta2)) + math.sqrt(((2*self.pulleyJointDifference*math.cos(math.pi - theta1 - theta2))**2) - (4*(-(winchLength**2) + (self.pulleyJointDifference**2)))))/2
        x = L5*math.cos(theta1 + theta2)
        y = L5*math.sin(theta1 + theta2)
        return(x, y)
    '''
    
    def moveTo(self, x, y):
        L5, theta = self.getArmInverseKinematics(x, y)
        self.winch.reel(L5)
        self.shoulder.setAngle(theta)
    
    def brake(self):
        self.shoulder.brake()
        self.winch.brake()
    
    def zeroEncoders(self):
        self.shoulder.encoder.setPosition(0)
        self.winch.motor.setSelectedSensorPosition(0)
    
    def moveShoulder(self, speed):
        if abs(speed) > 0:
            self.shoulder.move(speed)
        else:
            self.shoulder.brake()
    
    def setShoulderAngle(self, angle):
        self.shoulder.setAngle(angle)
    
    def setWinchPosition(self, position):
        self.winchPID.setSetpoint(position)
        
        speed = self.winchPID.calculate(self.winch.getAngle())
        if speed > 0:
            self.winch.forward(speed)
        elif speed < 0:
            self.winch.backward(speed)
        else:
            self.winch.brake()
    
    def moveWinch(self, speed):
        if abs(speed) > 0:
            self.winch.spin(speed)
        else:
            self.winch.brake()

class Shoulder:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, sparkID, name, config):
        self.motor = rev.CANSparkMax(sparkID, rev.CANSparkMax.MotorType.kBrushless)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.currentLimit = config["Climber"]["shoulderCurrentLimit"] # This amperage limit has not been tested
        self.velocityLimit = config["Climber"]["shoulderStressedVelocityThreshold"] # This number is for checking the current spike when motor stalls
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0)
        #self.motor.FaultID.kSoftLimitRev = 5  # This is in rotations (with a 100 to 1 gear ratio), and prevents backwards movement beyond the specified encoder value
        #self.motor.FaultID.kSoftLimitFwd = 60

        self.name = name
        if self.name == "leftShoulder":
            self.motor.setInverted(True)
            pVal = 0.008
            IvAL = 0.0003
        else:
            pVal = 0.008
            IvAL = 0.0003
            self.motor.setInverted(False)
        self.shoulderPID = wpimath.controller.PIDController(pVal, IvAL, -0.0001)
        self.shoulderPID.setTolerance(0.5, 5)

    def zeroShoulder(self):
        self.encoder.setPosition(0)
    def move(self, speed):
        self.motor.set(speed)
        
        '''if self.motor.getFault(rev.CANSparkMax.FaultID.kSoftLimitFwd):
            wpilib.SmartDashboard.putBool(self.name + " forward stop", True)
        else:
            wpilib.SmartDashboard.putBool(self.name + " forward stop", False)

        if self.motor.getFault(rev.CANSparkMax.FaultID.kSoftLimitRev):
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", True)
        else:
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", False)'''
            
    def setAngle(self, angle):
        #print(f"Name: {self.name}, Angle: {self.getAngle()}, Target Angle: {angle}")
        self.shoulderPID.setSetpoint(angle)
        speed = self.shoulderPID.calculate(self.getAngle())
        print(speed)
        if angle < 30:
            #print("below threshold")
            self.motor.set(0)
            self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        elif angle > 180:
            #print("above threshold")
            self.motor.set(0)
            self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        elif self.shoulderPID.atSetpoint():
            #print("at setpoint")
            self.motor.set(0)
            self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        else:
            self.motor.set(speed)
            #print(f"Name: {self.name}, Angle: {self.getAngle()}, Target Angle: {angle}, Motor Output: {speed}")
    def setVoltage(self, voltage):
        self.motor.setVoltage(voltage)
    def getAngle(self):
        angle = (self.encoder.getPosition() * 360 / 100) + 30 # 30 represents the idle arm angle minumum
        return(angle)
    
    def brake(self):
        self.motor.set(0)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        
    def coast(self):
        self.motor.set(0)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

class Winch:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, ID, name, config):
        self.config = config
        self.name = name
        if self.name == 'leftWinch':
            self.neo = rev.CANSparkMax(ID, rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless)
            self.neoEncoder = self.neo.getEncoder()
        else:    
            self.falcon = ctre.TalonFX(ID)
        self.currentLimit = config["Climber"]["winchCurrentLimit"] # This amperage limit has not been tested
        self.velocityLimit = config["Climber"]["winchStressedVelocityThreshold"] # This number is for checking the current spike when motor stalls
        #self.motor.configReverseSoftLimitThreshold(100) # This is in encoder ticks, and it prevents backwards movement beyond the specified encoder value
        #self.motor.configForwardSoftLimitThreshold(20480) # I set this limit to 10 rotations, but it just needs to be adjusted so the winch doesn't let too much out and start wrapping the other way around the axle
        self.winchPID = wpimath.controller.PIDController(self.config["Climber"]["winchPID"]["kP"], self.config["Climber"]["winchPID"]["kI"], self.config["Climber"]["winchPID"]["kD"])
        self.winchPID.setTolerance(0.05, 1)
        if self.name == "leftWinch":
            self.neo.setInverted(False)
            self.neoEncoder.setPosition(0)
            self.brake()
            self.neoPID = self.neo.getPIDController()
            self.neoPID.setP(0.00012)
            self.neoPID.setI(0.0000008)
            self.neoPID.setD(0)
            self.neoPID.setFF(0)
            #self.neoPID.setOutputRange(0,0)
        else:
            self.falcon.setSelectedSensorPosition(0)
            self.brake()
            self.falcon.setInverted(True)
            self.falcon.config_kP(0, 0.00085)
            self.falcon.config_kI(0, 0)
            self.falcon.config_kD(0, 0)
            self.falcon.config_kF(0, 0)
        length = 0
        self.winchRotationsToDistanceList = []
        numberOfWraps = 10
        detailPerRotation = 360
        strapThickness = 0.035
        axleRadius = 1
        for i in range(1,numberOfWraps*detailPerRotation):
            i /= detailPerRotation
            currentRadius = math.ceil(numberOfWraps - i)*strapThickness + axleRadius
            length += currentRadius*2*math.pi/detailPerRotation
            self.winchRotationsToDistanceList.append((i, length))
    
    def winchRotationLookup(self, distance):
        '''This finds the number of rotations you need the winch to do (approximately) based on a distance you want the winch to let out.'''
        for i in self.winchRotationsToDistanceList:
            if i[1] > distance:
                index = self.winchRotationsToDistanceList.index(i)
                break
        a = self.winchRotationsToDistanceList[index - 1]
        b = self.winchRotationsToDistanceList[index]
        deltaDistance1 = b[1] - a[1]
        deltaDistance2 = distance - a[1]
        ratioDistance = deltaDistance2/deltaDistance1
        deltaRotation = b[0] - a[0]
        rotationValue = a[0] + (deltaRotation*ratioDistance)
        return(rotationValue)

    def spin(self, speed):
        if self.name == "leftWinch":
            self.neo.set(speed)
        else:
            self.falcon.set(ctre.ControlMode.PercentOutput, speed)

        '''if ctre.Faults.ForwardSoftLimit in self.motor.getFaults():
            wpilib.SmartDashboard.putBool(self.name + " forward stop", False)
        else:
            wpilib.SmartDashboard.putBool(self.name + " forward stop", True)

        if ctre.Faults.ReverseSoftLimit in self.motor.getFaults():
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", False)
        else:
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", True)'''
    
    def setRPM(self, rpm):
        ''' Takes in a rpm in rev/second '''
        if self.name == "leftWinch":
            adjustedRPM = rpm * 48
            self.neoPID.setReference(adjustedRPM, rev.CANSparkMax.ControlType.kVelocity, 0, 0)
            wpilib.SmartDashboard.putNumber("leftArmRPM", self.neoEncoder.getVelocity() / 48)
        else:
            adjustedRPM = rpm * 60 * 2048 / 10 # adjusting for gear ratio and ticks per 100ms
            self.falcon.set(ctre.ControlMode.Velocity, adjustedRPM)
            wpilib.SmartDashboard.putNumber("rightArmRPM", self.falcon.getSelectedSensorVelocity(0) * 60 * 10 / (60 * 2048))
    
    def reel(self, amount):
        if amount < 0:
            print("Cannot Retract Winch That Far!")
            self.brake()
        else:
            rotationTarget = self.winchRotationLookup(amount)
            self.winchPID.setSetpoint(rotationTarget)
            wpilib.SmartDashboard.putNumber("Rotations", self.getRotations())
            motorSpeed = self.winchPID.calculate(self.getRotations())
            if self.winchPID.atSetpoint():
                self.brake()
            else:   
                self.spin(motorSpeed)
            print(f"Position: {self.getRotations()}, Rotation Target: {rotationTarget}, MotorSpeed: {motorSpeed}")
    
    def getRotations(self):
        if self.name == "leftWinch":
            return self.neoEncoder.getPosition() / 48
        else:
            return self.falcon.getSelectedSensorPosition() / (2048 * 60)
    
    def brake(self):
        if self.name == "leftWinch":
            self.neo.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
            self.neo.set(0)
        else:
            self.falcon.set(ctre.ControlMode.PercentOutput, 0)
            self.falcon.setNeutralMode(ctre.NeutralMode.Brake)
