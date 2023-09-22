from pymycobot import PI_PORT, PI_BAUD      # When using the Raspberry Pi version of mycobot, you can refer to these two variables to initialize MyCobot, if not, you can omit this line of code
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot(PI_PORT, PI_BAUD)

print(mc.get_angles())
