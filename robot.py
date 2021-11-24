'''
GOALS:
    - Have an easily customizable autonomous mode that can take coordinate inputs and do those movements
        -- python GUI app that outputs vector data and autonomous "map" into a csv
        -- autonomous reads a csv
        -- Figure out how to actually go an exact distance without guessing & checking times
            --- Can be done using circumference of wheels
    - Working swerve drive
    - Configuration file (preferabbly json) for most if not all robot settings
    - Communicate with vision using networktables
    - Get motor framework down (assume we are using talonfx controllers)

Installing Dependencies:
py -3 -m pip install -U robotpy[ctre, navx]
'''
import wpilib
import json
import os


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.left_motor = wpilib.Spark(0)
        self.right_motor = wpilib.Spark(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.stick = wpilib.Joystick(1)
        self.timer = wpilib.Timer()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        auto_plan = wpilib.SmartDashboard.getString("Auto Plan", "default")
        # self.auto = autonomousController(auto_plan)
        # self.timer.reset()
        # self.timer.start()
        self.auto.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            self.drive.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
        else:
            self.drive.arcadeDrive(0, 0)  # Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())

    def disabledPeriodic(self) -> None:
        return super().disabledPeriodic()

if __name__ == "__main__":
    wpilib.run(MyRobot)