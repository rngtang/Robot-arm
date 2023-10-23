from flask import Blueprint, current_app
import time
import sys

# Creates the blueprint
default = Blueprint('default', __name__)

# for lights and dance (asynch)
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

@default.route("/")
def default_pos():
    controls = current_app.config['controls']
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    mc.set_color(255, 255, 255) # start white
    time.sleep(1)
    return '''<h1>default </h1>'''