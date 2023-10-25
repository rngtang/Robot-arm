
from pymycobot import PI_PORT, PI_BAUD      # When using the Raspberry Pi version of mycobot, you can refer to these two variables to initialize MyCobot, if not, you can omit this line of code
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot(PI_PORT, PI_BAUD)


#loop 7 times
count = 7
while count > 0:
  mc.set_color(0,0,255) #blue light on
  time.sleep(2)    #wait for 2 seconds
  mc.set_color(255,0,0) #red light on
  time.sleep(2)    #wait for 2 seconds
  mc.set_color(0,255,0) #green light on
  time.sleep(2)    #wait for 2 seconds
  count -= 1
