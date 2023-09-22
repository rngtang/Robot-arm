from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord
import time
import random

mc = MyCobot("/dev/ttyAMA0", 1000000)

mc.send_angles([0, 0, 0, 0, 0, 0], 50)
time.sleep(3)
mc.set_color(0,0,255) #blue light on
# mc.set_color(255,0,0) #red light on

