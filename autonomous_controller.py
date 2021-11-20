import threading
import wpilib
import math
class autonomousController():
    def __init__(self, theBigPlay, wheelDiameter) -> None:
        # autoThread = threading.Thread(target=start())
        self.timer = wpilib.Timer()
        self.circumference = 2 * math.pi * (wheelDiameter / 2)
        x_coordinates = [] # import all 3 of these lists based off of an imported pandas csv
        y_coordinates = []
        robot_rotation = []
        steps = 0
        self.distances = []
        self.wheel_angles = []
        while (steps < x_coordinates.__len__()):
            x, y = x_coordinates[steps], y_coordinates[steps]
            hypotenuse = math.sqrt((x*x)+(y*y))
            wheel_angle = math.degrees(math.tan(x/y))
            self.distances.append(hypotenuse)
            self.wheel_angles.append(wheel_angle)
        # now figure out what commands and extra math to do to make go a certain distance
    
    def start(self):
        self.timer.start()
        # pseudocode
        '''
        zero motors
        set current angle to 0
        '''