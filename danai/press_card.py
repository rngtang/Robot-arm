from pymycobot import PI_PORT, PI_BAUD      # When using the Raspberry Pi version of mycobot, you can refer to these two variables to initialize MyCobot, if not, you can omit this line of code
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot(PI_PORT, PI_BAUD)


mc.send_angles([-33.31, -142.11, 42.45, 14.5, -7.03, -14.58], 80)
