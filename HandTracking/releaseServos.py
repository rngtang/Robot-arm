from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord
import time
import math

mc = MyCobot("/dev/ttyAMA0", 1000000)
mc.release_all_servos()
# mc.send_angles([0, 0, 0, 0, 0, 0], 40)
# time.sleep(10)
# coords = mc.get_coords()
# # time.sleep(8)
# print(math.sqrt(coords[0]**2 + coords[1]**2))


# mc.send_angles([0, -90, 0, 0, 0, 0], 40)
# time.sleep(12)
# coords = mc.get_coords()
# time.sleep(8)
# print(math.sqrt(coords[0]**2 + coords[1]**2) - 90)
while True:
    # coords = mc.get_coords()
    angles = mc.get_angles()
    # if len(coords) > 0 and len(angles) > 0:
    if len(angles) > 0:
        # print(max(0, math.sqrt(coords[0]**2 + coords[1]**2) - 110))
        # print(angles[1],",", angles[2], ",", angles[1] + angles[2])
        # if abs(angles[1]) > 90:

        print(abs(-1 * (abs(angles[1]) / 90) + 1))
