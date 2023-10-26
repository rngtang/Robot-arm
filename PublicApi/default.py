from flask import Blueprint, current_app
import time

# Creates the blueprint
default = Blueprint('default', __name__)

@default.route("/")
def default_pos():
    # Gets the myCobot and controls object from the app instance
    mc = current_app.config['mc']
    controls = current_app.config['controls']
    # Sends robot to the default position
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    mc.set_color(255, 255, 255) # start white
    time.sleep(1)
    return '''<h1>default </h1>''', 200