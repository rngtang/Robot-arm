# code from Elephant Robotics: https://docs.elephantrobotics.com/docs/gitbook-en/7-ApplicationBasePython/7.7_example.html

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # # When using the Raspberry Pi version of mycobot, these two variables can be referenced to initialize MyCobot
import RPi.GPIO as GPIO
import time

# MyCobot class initialization requires two parameters:
#   The first is the serial port string, such as:
#       linux: "/dev/ttyUSB0"
#          or "/dev/ttyAMA0"
#       windows: "COM3"
#   The second is the baud rate::
#       M5 version is: 115200
#
#    such as:
#       mycobot-M5:
#           linux:
#              mc = MyCobot("/dev/ttyUSB0", 115200)
#          or mc = MyCobot("/dev/ttyAMA0", 115200)
#           windows:
#              mc = MyCobot("COM3", 115200)
#       mycobot-raspi:
#           mc = MyCobot(PI_PORT, PI_BAUD)
#
# Initialize a MyCobot object
# Create object code here for Raspberry Pi version
mc = MyCobot(PI_PORT, PI_BAUD)

# The position of the robot arm movement
angles = [
            [92.9, -10.1, -60, 5.8, -2.02, -37.7],
            [92.9, -53.7, -83.05, 50.09, -0.43, -38.75],
            [92.9, -10.1, -87.27, 5.8, -2.02, -37.7]
        ]
# Initialize the suction pump
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

# Turn on the suction pump
def pump_on():
    # Make the 20 work
    GPIO.output(20，0)
    # Make the 21 work
    GPIO.output(21，0)

# Stop the suction pump
def pump_off():
    # Stop the 20 from working
    GPIO.output(21，1)
    # Stop the 21 from working
    GPIO.output(21，1)


# Robotic arm recovery
mc.send_angles([0, 0, 0, 0, 0, 0], 30)
time.sleep(3)

# Turn on the suction pump
pump_on()
mc.send_angles(angles[2], 30)
time.sleep(2)

# absorb small blocks
mc.send_angles(angles[1], 30)
time.sleep(2)
mc.send_angles(angles[0], 30)
time.sleep(2)
mc.send_angles(angles[1], 30)
time.sleep(2)

# Turn off the suction pump
pump_off()
mc.send_angles(angles[0], 40)
time.sleep(1.5)
