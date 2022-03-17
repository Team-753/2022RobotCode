import wpilib
import math

class shootAndRun:
    def __init__(self) -> None:
        #self.queue = [["revShooter", 8], ["wait", 2.5], ["indexBall"], ["wait", 2], ["shooterOff"], ["turnTo", 180], ["intakeDown"], ["intakeOn"], ["move", (0, -0.5, 0), 1.5], ["stationary"], ["turnTo", 0], ["revShooter", 9], ["wait", 2], ["intakeOff"], ["wait", 1.5], ["indexStop"], ["shooterOff"], ["done"]]
        #self.queue = [["indexBall"], ["intakeDown"], ["intakeOn"], ["move", (0, -0.5, 0), 1.5], ["stationary"], ["wait", 2], ["indexStop"], ["intakeOff"], ["turnTo", 180], ["stationary"], ["revShooter", 9], ["wait", 2.5], ["indexBall"], ["wait", 3], ["indexStop"], ["shooterOff"], ["done"]]
        #self.queue = [["revShooter", 9], ["move", (0, 0.50, 0), 1.53], ["stationary"], ["wait", 1], ["indexBall"], ["wait", 2], ["indexStop"], ["shooterOff"], ["done"]]
        self.queue = [["revShooter", 4.5], ["wait", 2], ["indexBall"], ["wait", 2], ["move", (0.15, 0.50, 0), 2.5], ["stationary"], ["indexStop"], ["shooterOff"], ["done"]]
        self.queuePosition = 0
    def periodic(self):
        #print(self.queuePosition)
        if self.queuePosition == len(self.queue):
            return ["done"],
        else:
            return self.queue[self.queuePosition]