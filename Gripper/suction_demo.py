from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import time, sys
import RPi.GPIO as GPIO

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

# Sets the what is the numbering of the pins
GPIO.setmode(GPIO.BCM)
# Sets pin 20 as an output
GPIO.setup(20, GPIO.OUT)

# Turns the pump on
def pump_on():
    GPIO.output(20, 0)

# Turns the pump off
def pump_off():
    GPIO.output(20, 1)

pump_on()
time.sleep(3)
pump_off()