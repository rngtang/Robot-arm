from flask import Blueprint
import time
import sys

# Creates the blueprint
default = Blueprint('default', __name__)

# for lights and dance (asynch)
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

try: 
    # for ROS (self-made controls)
    sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
    from controls import Controls
    controls = Controls()
except: 
    print("FROM DEFAULT: could not access Controls()")

@default.route("/")
def default_pos():
    # try: 
    #     controls.send_angles([0, 0, 0, 0, 0, 0], 70) 
    # except: 
        # print("FROM DEFAULT: could not send angle from Controls")
    mc.set_color(255, 255, 255) # start white
    time.sleep(1)
    return '''<h1>default </h1>'''