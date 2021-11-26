import wpilib
import wpimath.trajectory as trajectory
from wpimath.kinematics import (SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState)
from threading import Thread

trajectoryUtilities = trajectory.TrajectoryUtil
path1 = trajectoryUtilities.serializeTrajectory(trajectoryUtilities.fromPathweaverJson("./pathweaverexample.json"))
Timer = wpilib.timer.Timer()
jsonFile = {}

'''
NOTE: Just thought of a mega-brained idea, instead of trying to follow a complicated digitally-determined path
we record all the commands and their parameters that are called to the drivetrain and subsequent subsystems whilst in 
a teleop path "recording" mode which can then be put into a large json file that can be better interpreted come autonomous.
While this may lead to less accuracy due to human error, this would make the process of creating, programming, and executing autonomous
plays much, MUCH less of a headache.
- An easy way to accomplish this is to simply record the output from the switches dictionary and index through the functions when looping through autonomous periodic.
- Keep in mind: TeleopPeriodic is ran about every 20ms to 50x a second and so is auto periodic
- Building on the first method mentioned is to instead of running new commands every 20ms, we instead should pick a time interval and run commands based on where the timer currently is and its corresponding dictionary.

'''

def autonomousGo():
    Timer.start()
    while Timer.get() < 25:
        i = 0
        for point in jsonFile:
            while not (Timer.get() >= point.get("time")): # there is a better function to go inside this if statement, check docs
                pass
            else:
                acceleration = point.get("acceleration")
                radiansPerMeter = point.get("curvature")
                velocity = point.get("velocity")
                time = point.get("time")
            i += 1

driveThread = Thread(None, target=autonomousGo())

'''
generate a path
make a bunch of points on the path with velocities and rotations and such
use swerve odometry to find robot location
find closest point on the path and give its instructions to the swerve kinematics
swerve kinematics gives instructions to the pid controllers that control the swerve modules
'''
