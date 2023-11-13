from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # # When using the Raspberry Pi version of mycobot, these two variables can be referenced to initialize MyCobot
import RPi.GPIO as GPIO
import time, sys

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

down = [15.98, -59.98, -78.91, 47.45, -1.23, 0]
default = [0, 0, 0, 0, 0, 0]

# Turns the pump on
def pump_on():
    GPIO.output(20, 0)
    GPIO.output(21, 0)

# Turns the pump off
def pump_off():
    GPIO.output(20, 1)
    GPIO.output(21, 1)

if __name__ == "__main__":

    # Sets the what is the numbering of the pins
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # Sets pins 20 and 21 as an output
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)

    controls.send_angles(default, 65)
    time.sleep(4.5)
    controls.send_angles(down, 65)
    time.sleep(2)
    pump_on()
    controls.send_angles(default, 65)
    time.sleep(3)
    pump_off()