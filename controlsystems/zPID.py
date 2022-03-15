import math
import wpimath.controller


class zPID:
    def __init__(self, navxOBJ) -> None:
        self.zPID = wpimath.controller.PIDController(0.0005, 0.0, 0)
        self.zPID.enableContinuousInput(-math.pi, math.pi)
        self.zPID.setTolerance(0, 0)
        self.navx = navxOBJ
    
    def periodic(self, targetAngle):
        targetAngle = targetAngle * math.pi / 180
        robotRotationRadians = self.navx.getAngle() * math.pi / 180
        z = self.zPID.calculate(targetAngle, robotRotationRadians)
        return z