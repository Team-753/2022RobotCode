import wpilib
import wpimath.trajectory as trajectory
from wpimath.kinematics import (SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState)
from threading import Thread

trajectoryUtilities = trajectory.TrajectoryUtil
path1 = trajectoryUtilities.serializeTrajectory(trajectoryUtilities.fromPathweaverJson("./pathweaverexample.json"))
Timer = wpilib.timer.Timer()
jsonFile = {}


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