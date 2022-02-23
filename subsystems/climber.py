import math
from turtle import speed
import wpilib
import ctre
import rev
import wpimath.controller

class Climber:
    def __init__(self, config: dict):
        self.config = config
        
        leftShoulder = Shoulder(self.config["Climber"]["leftShoulder"]["ID"], self.config["Climber"]["leftShoulder"]["Name"])
        rightShoulder = Shoulder(self.config["Climber"]["rightShoulder"]["ID"], self.config["Climber"]["rightShoulder"]["Name"])
        
        leftWinch = Winch(self.config["Climber"]["leftWinch"]["ID"], self.config["Climber"]["leftWinch"]["Name"])
        rightWinch = Winch(self.config["Climber"]["rightWinch"]["ID"], self.config["Climber"]["rightWinch"]["Name"])
        
        self.leftArm = Arm(leftWinch, leftShoulder, self.config, True)
        self.rightArm = Arm(rightWinch, rightShoulder, self.config, False)
        
        self.leftHook = wpilib.Solenoid(self.config["Climber"]["leftHook_PCM_ID"])
        self.rightHook = wpilib.Solenoid(self.config["Climber"]["rightHook_PCM_ID"])

        self.winchRotationsToDistanceList = self.generateWinchList(1, 0.0394, 5, 360)

        #self.shoulderPID = wpimath.controller.PIDController(self.config["Climber"]["shoulderPID"]["kP"], self.config["Climber"]["shoulderPID"]["kI"], self.config["Climber"]["shoulderPID"]["kD"])
        #self.winchPID = wpimath.controller.PIDController(self.config["Climber"]["winchPID"]["kP"], self.config["Climber"]["winchPID"]["kI"], self.config["Climber"]["winchPID"]["kD"])


    def generateWinchList(self, axleRadius, strapThickness, numberOfWraps, detailPerRotation):
        '''This generates a list of rotation amounts of the winch and their corresponding distance values that the winch lets out.'''
        length = 0
        aList = []
        for i in range(0,numberOfWraps*detailPerRotation):
            i /= detailPerRotation
            currentRadius = math.ceil(numberOfWraps - i)*strapThickness + axleRadius
            length += currentRadius*2*math.pi/detailPerRotation
            aList.append((i, length))
        return(aList)
    
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

    def getArmInverseKinematics(x, y):
        '''This returns the angle of the shoulder and the length of strap let out of the winch based on the desired x and y position of the arm hook.'''
        # TODO: Put these constants in the config file (because they might be wrong).
        L1 = 11
        L2 = 2.5
        L3 = 16.5
        L4 = math.hypot(L2, L3)
        L6 = 3
        theta = math.acos(x/(math.hypot(x,y))) - math.acos(((x**2)+(y**2)+(L1**2)-(L4**2))/(2*L1*math.hypot(x,y))) - (math.pi/4)
        theta = theta * 180 / math.pi
        L5 = math.hypot(x,(y-L6))
        return(L5, theta)

    def zeroEncoders(self):
        self.leftShoulder.encoder.setPosition(0)
        self.rightShoulder.encoder.setPosition(0)
        self.leftWinch.motor.setSelectedSensorPosition(0)
        self.rightWinch.motor.setSelectedSensorPosition(0)

    def setShoulderAngle(self, angle):
        self.shoulderPID.setSetpoint(angle)
        
        speed = self.shoulderPID.calculate(self.leftShoulder.getAngle())
        if speed > 0:
            self.leftShoulder.forward(speed)
        elif speed < 0:
            self.leftShoulder.backward(speed)
        else:
            self.leftShoulder.brake()
        
        speed = self.shoulderPID.calculate(self.rightShoulder.getAngle())
        if speed > 0:
            self.rightShoulder.forward(speed)
        elif speed < 0:
            self.rightShoulder.backward(speed)
        else:
            self.rightShoulder.brake()
    
    def moveShoulder(self, speed):
        if speed > 0:
            self.leftShoulder.forward(speed)
            self.rightShoulder.forward(speed)
        elif speed < 0:
            self.leftShoulder.backward(speed)
            self.rightShoulder.backward(speed)
        else:
            self.leftShoulder.brake()
            self.rightShoulder.brake()
    
    def setWinchPosition(self, position):
        self.winchPID.setSetpoint(position)
        
        speed = self.winchPID.calculate(self.leftWinch.getAngle())
        if speed > 0:
            self.leftWinch.forward(speed)
        elif speed < 0:
            self.leftWinch.backward(speed)
        else:
            self.leftWinch.brake()
        
        speed = self.winchPID.calculate(self.rightWinch.getAngle())
        if speed > 0:
            self.rightWinch.forward(speed)
        elif speed < 0:
            self.rightWinch.backward(speed)
        else:
            self.rightWinch.brake()
    
    def moveWinch(self, speed):
        if speed > 0:
            self.leftWinch.forward(speed)
            self.rightWinch.forward(speed)
        elif speed < 0:
            self.leftWinch.backward(speed)
            self.rightWinch.backward(speed)
        else:
            self.leftWinch.brake()
            self.rightWinch.brake()

    def extendArms(self):
        ''''''
    def firstBarGrab(self):
        ''''''
    def nextBarGrab(self):
        ''''''
    def pullArms(self):
        ''''''
    def detachArms(self):
        ''''''
    def engageHooks(self):
        ''''''
    def disengageHooks(self):
        ''''''

class Arm:
    def __init__(self, winch, shoulder, config) -> None:
        self.winch = winch
        self.shoulder = shoulder
        self.config = config
       
    def calculateTarget(self, x, y):
        lengthOne = 1
        lengthFour = 1
        lengthFive = math.sqrt((x**2) + (y**2))
        shoulderTargetTheta = math.acos(x / lengthFive) - math.acos(((x**2) + (y**2) + (lengthOne**2) - (lengthFour**2))/(2 * lengthOne * lengthFive))

class Shoulder:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, sparkID, name, config):
        self.motor = rev.CANSparkMax(sparkID, rev.CANSparkMax.MotorType.kBrushless)
        self.currentLimit = config["Climber"]["shoulderCurrentLimit"] # This amperage limit has not been tested
        self.velocityLimit = config["Climber"]["shoulderStressedVelocityThreshold"] # This number is for checking the current spike when motor stalls
        self.encoder = self.motor.getEncoder()
        self.motor.FaultID.kSoftLimitRev = 5 # This is in rotations (with a 100 to 1 gear ratio), and prevents backwards movement beyond the specified encoder value
        self.motor.FaultID.kSoftLimitFwd = 60
        self.name = name
        if self.name == "leftShoulder":
            self.motor.setInverted(True)
        else:
            self.motor.setInverted(False)

    def forward(self, speed):
        self.motor.set(speed)
    
    def backward(self, speed):
        self.motor.set(speed)
        if self.motor.getFault(rev.CANSparkMax.FaultID.kSoftLimitRev):
            wpilib.SmartDashboard.putBool(self.name + " stopped", True)
        else:
            wpilib.SmartDashboard.putBool(self.name + " stopped", False)
    
    def getAngle(self):
        angle = (self.encoder.getPosition() * 8.571) + 30 # 30 represents the idle arm angle minumum
        return(angle)
    
    def brake(self):
        self.motor.set(0)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

class Winch:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, talonID, name, config):
        self.motor = ctre.TalonFX(talonID)
        self.currentLimit = config["Climber"]["winchCurrentLimit"] # This amperage limit has not been tested
        self.velocityLimit = config["Climber"]["winchStressedVelocityThreshold"] # This number is for checking the current spike when motor stalls
        self.motor.setSelectedSensorPosition(0)
        self.motor.configReverseSoftLimitThreshold(100) # This is in encoder ticks, and it prevents backwards movement beyond the specified encoder value
        self.name = name
        self.CurrentCircumference = math.pi * 1 # idk what the base diameter of the pulley is
        self.winchPID = wpimath.controller.PIDController(self.config["Climber"]["winchPID"]["kP"], self.config["Climber"]["winchPID"]["kI"], self.config["Climber"]["winchPID"]["kD"])
        self.winchPID.setTolerance(0.5, 1)
        if self.name == "leftWinch":
            self.motor.setInverted(True)
        else:
            self.motor.setInverted(False)
        self.brake()

    def release(self, speed):
        self.motor.set(ctre.ControlMode.PercentOutput, speed)

    def retract(self, speed):
        self.motor.set(ctre.ControlMode.PercentOutput, speed)
        if ctre.Faults.ReverseSoftLimit in self.motor.getFaults():
            wpilib.SmartDashboard.putBool(self.name + " stopped", False)
        else:
            wpilib.SmartDashboard.putBool(self.name + " stopped", True)
    
    def reel(self, amount):
        if amount < 0:
            print("Cannot Retract Winch That Far!")
            self.brake()
        else:
            self.winchPID.setSetpoint(amount)
            motorSpeed = self.winchPID.calculate(self.getWinchLength())
            if self.winchPID.atSetpoint():
                self.brake()
            else:   
                self.motor.set(ctre.ControlMode.PercentOutput, motorSpeed)
    
    def getWinchLength(self):
        position = self.motor.getSelectedSensorPosition()
        length = position * 0.17578125 * self.CurrentCircumference
        return(length)
    
    def brake(self):
        self.motor.set(ctre.ControlMode.PercentOutput, 0)
        self.motor.setNeutralMode(ctre.NeutralMode.Brake)
