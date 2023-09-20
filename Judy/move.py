# Moves the robot to the coordinates

from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

# The positions are defined as followed
board = [
	[ '1,1', '1,2', '1,3' ],
	[ '2,1', '2,2', '2,3' ],
	[ '3,1', '3,2', '3,3' ]
]

mc = MyCobot("/dev/ttyAMA0", 115200)

# mc.send_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(3)
mc.set_color(0,0,255) #blue light on
print('test')

# def movePos(position):
