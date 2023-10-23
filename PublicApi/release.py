from flask import Blueprint, current_app
import time
import sys

# Creates the blueprint
release = Blueprint('release', __name__)

# for lights and dance (asynch)
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

@release.route("/")
def release_joints():
    controls = current_app.config['controls']
    controls.toggle_servo_release(True)
    time.sleep(1)
    return '''<h1>release </h1>'''