from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import sys
import termios
import tty
import time
import RPi.GPIO as GPIO

msg = """
With this teleop program you will be able to move all of the 6 joints using the keyboard
To control the arm use the following commands:

joint 1: Q and A
joint 2: W and S
joint 3: E and D
joint 4: R and F
joint 5: T and G
joint 6: Y and H

Press B to print the current angle of each of the joints press

Press O to reset the arm to the default position

"""

# Takes input from keyboard
class Raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

arm = MyCobot("/dev/ttyAMA0", 1000000)

# Initialize
GPIO.setmode(GPIO.BCM)
# Either pin 20/21 can control the switch of the suction pump. Note: the switch should use the same pin Foot control
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

# Turn on the suction pump
def pump_on():
    GPIO.output(23, 0)
    GPIO.output(24, 0)

# Stop suction pump
def pump_off():
    GPIO.output(23, 1)
    GPIO.output(24, 1)

# Current position of the joints
j1 = 0
j2 = 0
j3 = 0
j4 = 0
j5 = 0
j6 = 0
# How many angles per iteration will change
change = 1
# Moving arm to default position
print("Resetting position to default...")
arm.send_angles([0, 0, 0, 0, 0, 0], 70)

print(msg)

while True:
    with Raw(sys.stdin):
        key = sys.stdin.read(1)
    if key == 'b':
        print("\nCurrent angles: [j1: " + str(j1) + ", j2: " + str(j2), ", j3: " + str(j3) + ", j4: " + str(j4) + ", j5: " + str(j5) + ", j6: " + str(j6) + "]")
    if key == 'o':
        j1 = 0
        j2 = 0
        j3 = 0
        j4 = 0
        j5 = 0
        j6 = 0
        print("\n")
        arm.send_angles([0, 0, 0, 0, 0, 0], 70)
    if key == 'q':
        j1+=change
        if j1 > 165: j1 = 165
        arm.send_angle(1, j1, 90)
    if key == 'a':
        j1 -= change
        if j1 < -165: j1 = -165
        arm.send_angle(1, j1, 10)
    if key == 'w':
        j2+=change
        if j2 > 165: j2 = 165
        arm.send_angle(2, j2, 90)
    if key == 's':
        j2 -= change
        if j2 < -165: j2 = -165
        arm.send_angle(2, j2, 10)
    if key == 'e':
        j3+=change
        if j3 > 165: j3 = 165
        arm.send_angle(3, j3, 90)
    if key == 'd':
        j3 -= change
        if j3 < -165: j3 = -165
        arm.send_angle(3, j3, 10)
    if key == 'r':
        j4+=change
        if j4 > 165: j4 = 165
        arm.send_angle(4, j4, 90)
    if key == 'f':
        j4 -= change
        if j4 < -165: j4 = -165
        arm.send_angle(4, j4, 10)
    if key == 't':
        j5+=change
        if j5 > 165: j5 = 165
        arm.send_angle(5, j5, 90)
    if key == 'g':
        j5 -= change
        if j5 < -165: j5 = -165
        arm.send_angle(5, j5, 10)
    if key == 'y':
        j6+=change
        if j6 > 165: j6 = 165
        arm.send_angle(6, j6, 90)
    if key == 'h':
        j6 -= change
        if j6 < -165: j6 = -165
        arm.send_angle(6, j6, 10)
    if key == 'p':
        pump_on()
    if key == 'l':
        pump_off()