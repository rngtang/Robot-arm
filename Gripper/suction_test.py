from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of mycobot, you can refer to these two variables to initialize MyCobot
import time
import RPi.GPIO as GPIO

# Initialize a MyCobot object 
mc = MyCobot(PI_PORT, PI_BAUD)

# Initialize
GPIO.setmode(GPIO.BCM)
# Either pin 20/21 can control the switch of the suction pump. Note: the switch should use the same pin Foot control
GPIO.setup(20, GPIO.OUT)
#GPIO.setup(21, GPIO.OUT)

# Turn on the suction pump
def pump_on():
     # open suction pump
    GPIO.output(20，0)
    #GPIO.output(21，0)

# Stop suction pump
def pump_off():
    # Shut down the suction pump
    GPIO.output(20，1)
    #GPIO.output(21，1)

pump_off()
time.sleep(3)
pump_on()
time.sleep(3)
pump_off()
time.sleep(3)