# Docs: https://robotpy.readthedocs.io/en/stable/guide/nt.html#client-initialization-driver-station-coprocessor
import threading
from networktables import NetworkTables
import time
import board
import neopixel
import RPi.GPIO as GPIO
import logging

logging.basicConfig(level=logging.DEBUG)

GPIO.setmode(GPIO.BCM)
num_pixels = 3
ORDER = neopixel.RGBW
pixel_pin = board.D18 # Neo pixel digital out connected to the 6th pin down on the second column
brightness_ = 0.1
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=brightness_, auto_write=False, pixel_order=ORDER)
pixels.fill((255, 0, 0, 0))
pixels.show()
cond = threading.Condition()
NetworkTables.initialize(server='roborio-753-frc.local') # roborio-753-frc.local
sd = NetworkTables.getTable("SmartDashboard")

# Our main code goes here

def main():
    limelight = sd.getAutoUpdateValue('aether', False) # First parameter is the name of the instance, the second is the default value if nothing is found.
    robotEnabled = sd.getAutoUpdateValue('robotEnabled', False)
    startTime = time.perf_counter()
    while True:
        timeElapsed = time.perf_counter() - startTime
        if NetworkTables.isConnected():
            if robotEnabled.value:
                if limelight.value:
                    pixels.brightness = 1.0
                    pixels.fill((0, 255, 0, 0))
                else:
                    pixels.fill((0, 0, 0, 0))
            else:
                pixels.brightness = 0.05
                if (timeElapsed % 2) >= 1:
                    pixels.fill((0, 0, 255, 0))
                    print("nt connected")
                    print(NetworkTables.isConnected())
                else:
                    pixels.fill((0, 0, 0, 0))
        else:
            pixels.brightness = 0.05
            if (timeElapsed % 2) >= 1:
                pixels.fill((255, 0, 0, 0))
                print("nt disconnected")
                print(NetworkTables.isConnected())
            else:
                pixels.fill((0, 0, 0, 0))
        pixels.show()
        time.sleep(0.02)

main()
'''if __name__ == '__main__':
    try:
        main()
    except:
        print("exception; exiting program")
        pixels.fill((0, 0, 0, 0))
        GPIO.cleanup()'''