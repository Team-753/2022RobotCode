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
import driveTrain
import driverStation
import autonomous


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        with open(f"{os.getcwd()}/config.json", "r") as f1:
            self.config = json.load(f1)
        self.timer = wpilib.Timer()
        self.driveTrain = driveTrain.driveTrain(self.config)
        self.driverStation = driverStation.driverStation(self.config)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        auto_plan = wpilib.SmartDashboard.getString("Auto Plan", "getOffLine")
        # self.auto = autonomousController(auto_plan)
        # self.timer.reset()
        # self.timer.start()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
        if self.timer.get() < 2.0:
            self.drive.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
        else:
            self.drive.arcadeDrive(0, 0)  # Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""

    def disabledPeriodic(self):
        ''' Intended to update shuffleboard with drivetrain values used for zeroing '''
        pass
    
    def testInit(self) -> None:
        pass
    
    def testPeriodic(self):
        ''' My intention with this function is to have the robot automatically run through
            all of its subsystems and functions similar to autonomous but much more limited.'''
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)