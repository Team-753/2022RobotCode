import math
import wpilib
import ctre
import rev
import wpimath.controller

class Climber:
    def __init__(self, config: dict):
        self.config = config
        
        self.leftShoulder = Shoulder(self.config["Climber"]["leftShoulder"]["ID"], self.config["Climber"]["leftShoulder"]["Name"], self.config)
        self.rightShoulder = Shoulder(self.config["Climber"]["rightShoulder"]["ID"], self.config["Climber"]["rightShoulder"]["Name"], self.config)

        self.leftWinch = Winch(self.config["Climber"]["leftWinch"]["ID"], self.config["Climber"]["leftWinch"]["Name"], self.config)
        self.rightWinch = Winch(self.config["Climber"]["rightWinch"]["ID"], self.config["Climber"]["rightWinch"]["Name"], self.config)

        self.leftArm = Arm(self.leftWinch, self.leftShoulder, self.config)
        self.rightArm = Arm(self.rightWinch, self.rightShoulder, self.config)
        
        self.leftHook = wpilib.DoubleSolenoid(self.config["PCM"], wpilib.PneumaticsModuleType.CTREPCM, forwardChannel = self.config["Climber"]["leftHook_PCM_ID_Forward"], reverseChannel = self.config["Climber"]["leftHook_PCM_ID_Reverse"])
        self.rightHook = wpilib.DoubleSolenoid(self.config["PCM"], wpilib.PneumaticsModuleType.CTREPCM, forwardChannel = self.config["Climber"]["rightHook_PCM_ID_Forward"], reverseChannel = self.config["Climber"]["rightHook_PCM_ID_Reverse"])

        #self.winchRotationsToDistanceList = self.generateWinchList(1, 0.0394, 5, 360) # TODO: change the number of wraps (third parameter) to whatever the amount actually is.

        self.shoulderPID = wpimath.controller.PIDController(self.config["Climber"]["shoulderPID"]["kP"], self.config["Climber"]["shoulderPID"]["kI"], self.config["Climber"]["shoulderPID"]["kD"])
        self.winchPID = wpimath.controller.PIDController(self.config["Climber"]["winchPID"]["kP"], self.config["Climber"]["winchPID"]["kI"], self.config["Climber"]["winchPID"]["kD"])


    def zeroEncoders(self):
        self.leftShoulder.encoder.setPosition(0)
        self.rightShoulder.encoder.setPosition(0)
        self.leftWinch.motor.setSelectedSensorPosition(0)
        self.rightWinch.motor.setSelectedSensorPosition(0)

    def setShoulderAngles(self, angle):
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
    
    def moveShoulders(self, speed):
        if abs(speed) > 0:
            self.leftShoulder.move(speed)
            self.rightShoulder.move(speed)
        else:
            self.leftShoulder.brake()
            self.rightShoulder.brake()
    
    def setWinchPositions(self, position):
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
    
    def moveWinches(self, speed):
        if abs(speed) > 0:
            self.leftWinch.move(speed)
            self.rightWinch.move(speed)
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
       
    def getArmInverseKinematics(self, x, y):
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
        
    def moveTo(self, x, y):
        L5, theta = self.getArmInverseKinematics(x, y)
        self.winch.reel(L5)
        self.shoulder.setAngle(theta)

class Shoulder:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, sparkID, name, config):
        self.motor = rev.CANSparkMax(sparkID, rev.CANSparkMax.MotorType.kBrushless)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.currentLimit = config["Climber"]["shoulderCurrentLimit"] # This amperage limit has not been tested
        self.velocityLimit = config["Climber"]["shoulderStressedVelocityThreshold"] # This number is for checking the current spike when motor stalls
        self.encoder = self.motor.getEncoder()
        #self.motor.FaultID.kSoftLimitRev = 5 # This is in rotations (with a 100 to 1 gear ratio), and prevents backwards movement beyond the specified encoder value
        #self.motor.FaultID.kSoftLimitFwd = 60
        self.shoulderPID = wpimath.controller.PIDController(0.005, 0, 0)
        self.shoulderPID.setTolerance(1, 1)
        self.name = name
        if self.name == "leftShoulder":
            self.motor.setInverted(True)
        else:
            self.motor.setInverted(False)

    def move(self, speed):
        self.motor.set(speed)
        
        if self.motor.getFault(rev.CANSparkMax.FaultID.kSoftLimitFwd):
            wpilib.SmartDashboard.putBool(self.name + " forward stop", True)
        else:
            wpilib.SmartDashboard.putBool(self.name + " forward stop", False)

        if self.motor.getFault(rev.CANSparkMax.FaultID.kSoftLimitRev):
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", True)
        else:
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", False)
            
    def setAngle(self, angle):
        if angle < 30:
            self.motor.set(0)
            self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        if angle > 180:
            self.motor.set(0)
            self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        if self.shoulderPID.atSetpoint():
            self.motor.set(0)
            self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        else:
            self.shoulderPID.setSetpoint(angle)
            speed = self.shoulderPID.calculate(self.getAngle())
            self.motor.set(speed)
    
    def getAngle(self):
        angle = ((self.encoder.getPosition() * (360 / 42)) / 100) + 30 # 30 represents the idle arm angle minumum
        return(angle)
    
    def brake(self):
        self.motor.set(0)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        
    def coast(self):
        self.motor.set(0)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

class Winch:
    '''The forward and backward directions need to be tested.'''
    def __init__(self, talonID, name, config):
        self.config = config
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
        length = 0
        self.winchRotationsToDistanceList = []
        numberOfWraps = 0
        detailPerRotation = 0
        strapThickness = 0
        axleRadius = 0.5
        for i in range(0,numberOfWraps*detailPerRotation):
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

    def release(self, speed):
        self.motor.set(ctre.ControlMode.PercentOutput, speed)

        if ctre.Faults.ForwardSoftLimit in self.motor.getFaults():
            wpilib.SmartDashboard.putBool(self.name + " forward stop", False)
        else:
            wpilib.SmartDashboard.putBool(self.name + " forward stop", True)

        if ctre.Faults.ReverseSoftLimit in self.motor.getFaults():
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", False)
        else:
            wpilib.SmartDashboard.putBool(self.name + " reverse stop", True)
    
    def reel(self, amount):
        if amount < 0:
            print("Cannot Retract Winch That Far!")
            self.brake()
        else:
            rotationTarget = self.winchRotationLookup(amount)
            self.winchPID.setSetpoint(rotationTarget)
            motorSpeed = self.winchPID.calculate(self.winchRotationLookup(self.getWinchLength()))
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
