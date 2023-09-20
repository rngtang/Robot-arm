from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle, Coord
from pymycobot import PI_PORT, PI_BAUD
import time

# Base coordinates: [8.9, -62.6, 416.2, -80.59, 0.14, -80.3]


mc = MyCobot("/dev/ttyAMA0", 1000000)

mc.send_angles([0, 0, 0, 0, 0, 0], 50)
mc.send_coord(Coord.X.value, 50, 50)

time.sleep(1)

coords = mc.get_coords()
print(coords)

# coords[0] += 50
# # mc.sync_send_coords(coords, 50, 0)
# print(coords)

mc.release_all_servos()
time.sleep(1)