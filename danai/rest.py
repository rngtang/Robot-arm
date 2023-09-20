from pymycobot import PI_PORT, PI_BAUD      # When using the Raspberry Pi version of mycobot, you can refer to these two variables to initialize MyCobot, if not, you can omit this line of code
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot(PI_PORT, PI_BAUD)

mc.send_angles([88.68, -138.51, 155.65, -128.05, -9.93, -15.29], 50)
time.sleep(2.5)
mc.release_all_servos()
