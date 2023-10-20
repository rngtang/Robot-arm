from flask import Blueprint
import time
import sys

# for lights and dance (asynch)
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

# for ROS (self-made controls)
try:
    sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
    from controls import Controls

    default = Blueprint('default', __name__)
    controls = Controls()
except Exception:
    print("FROM APP: can't connect to controls")

# Creates the blueprint
# default = Blueprint('default', __name__)

@default.route("/")
def default_pos(): 
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    mc.set_color(255, 255, 255) # start white
    time.sleep(1)
    return '''<h1>default </h1>'''