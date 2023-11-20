from flask import Blueprint, current_app
import time

# Creates the blueprint
default = Blueprint('default', __name__)

# testing -> why can't it be called as a config ? 
# from pymycobot.mycobot import MyCobot
# mc = MyCobot("/dev/ttyAMA0", 1000000)

@default.route("/")
def default_pos():
    # Gets the myCobot and controls object from the app instance
    mc = current_app.config['mc']
    lock = current_app.config['lock']
    if lock.locked():
        return "A request is already in progress"

    with lock:
        try: 
            controls = current_app.config['controls']
        except: 
            return '''<h1>Unable to connect to Controls</h1>''', 500

        # Sends robot to the default position
        controls.send_angles([0, 0, 0, 0, 0, 0], 70)
        time.sleep(1)
        mc.set_color(255, 255, 255) # start white
        time.sleep(1)
        return '''<h1>default </h1>''', 200