from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord

import RPi.GPIO as GPIO
import time

mc = MyCobot("/dev/ttyAMA0", 1000000)

GPIO.setmode(GPIO.BCM)
GPIO.setup(1, GPIO.OUT)
# GPIO.setup(21, GPIO.OUT)


while True:
    print("Choose an option:")
    print("1. Turn on pump")
    print("2. Turn off pump")
    print("3. Exit")
    choice = input("Enter your choice: ")

    if choice == '1':
        print("Pump on")
        GPIO.output(1, 0)
        # GPIO.output(21, 0)
    elif choice == '2':
        print("Pump off")
        GPIO.output(1, 1)
        # GPIO.output(21, 1)
    elif choice == '3':
        break
    else:
        print("Invalid choice. Please enter 1, 2, or 3.")

GPIO.cleanup()