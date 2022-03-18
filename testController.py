from inputs import get_gamepad
import math
import threading
import pygame

pygame.init()

sign = lambda x: math.copysign(1, x) 

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def deadzone(self, x):
        if abs(x) < 0.1:
            x = 0
        else:
            x = sign(x)*(abs(x)-0.1)/0.9
        return x

    def read(self):
        '''
        These are all the buttons on the driver controller that control input to the drivetrain. 
        The deadzone is the same for every input but they will probably need to be different on the real robot.
        '''
        x = self.LeftJoystickX
        x = self.deadzone(x)
        y = self.LeftJoystickY
        y = self.deadzone(y)
        z = self.RightJoystickX
        z = self.deadzone(z)
        v1 = self.RightTrigger # Velocity multiplier for translation which increases speed
        v1 = self.deadzone(v1)
        v2 = self.RightBumper # Velocity multiplier for translation which decreases speed
        r = self.B # Resets the robot back to middle (it can get lost off screen)
        w1 = self.LeftBumper # Velocity multiplier for rotation which decreases speed
        return [x, y, z, v1, v2, r, w1]


    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.X = event.state
                elif event.code == 'BTN_WEST':
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state


class Screen:
    def __init__(self, controller):
        self.screenSize = (1280, 720)
        self.screen = pygame.display.set_mode(self.screenSize)
        self.screen.fill("black")
        self.controller = controller

    def start(self):
        values = []
        angle1 = math.pi/4
        position = [0,0]
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
            if joy.Back == 1 and joy.Start == 1:
                pygame.quit()
                exit()
            if values != joy.read():
                values = joy.read()
            
            '''
            Here are where the inputs are processed. The speed of the robot's translation and the speed of the robot's rotation can be changed by different inputs on the controller.
            In this case, the left bumper slows down the turning, the right bumper slows down the driving, the right trigger speeds up the driving.
            Also, the right joystick x controls the rotation and the left joystick x and y control the translation x and y. 
            Another step in adjusting the controls is accounting for drift in the rotation of the robot while it drives. The robot should only turn as much as the driver wants.
            A PID will need to adjust the z input until the robot is turning as much as it is told to turn.
            '''

            speed = values[3] + 1
            speed /= 2*values[4] + 1

            angularSpeed = 1/(2*values[6]+1)

            angle1 -= angularSpeed*values[2]/100

            position[0] += speed*values[0]
            position[1] -= speed*values[1]

            if values[5] == 1:
                position = [0,0]
            
            self.screen.fill("black")
            pygame.draw.circle(self.screen, "red", ((position[0] + self.screenSize[0]/2) + 90*math.cos(angle1), (position[1] + self.screenSize[1]/2) - 78*math.sin(angle1)), 4)
            pygame.draw.circle(self.screen, "white", ((position[0] + self.screenSize[0]/2) + 90*math.cos(angle1+math.pi/2), (position[1] + self.screenSize[1]/2) - 78*math.sin(angle1+math.pi/2)), 4)
            pygame.draw.circle(self.screen, "white", ((position[0] + self.screenSize[0]/2) + 90*math.cos(angle1+math.pi), (position[1] + self.screenSize[1]/2) - 78*math.sin(angle1+math.pi)), 4)
            pygame.draw.circle(self.screen, "white", ((position[0] + self.screenSize[0]/2) + 90*math.cos(angle1+3*math.pi/2), (position[1] + self.screenSize[1]/2) - 78*math.sin(angle1+3*math.pi/2)), 4)
            pygame.display.update()

            print(angle1)

if __name__ == '__main__':
    joy = XboxController()
    screen1 = Screen(joy)
    screen1.start()