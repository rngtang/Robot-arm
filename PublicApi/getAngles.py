from flask import Blueprint, current_app
import time

g_angles = Blueprint('getAngles', __name__)
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

@g_angles.route("/")
def getAngles():
    controls = current_app.config['controls']
    # Tries to catch any errors
    try:
        angles = mc.get_angles()
        return '''<h1>Joint 1: {0}<br>
        Joint 2: {1}<br>
        Joint 3: {2}<br>
        Joint 4: {3}<br>
        Joint 5: {4}<br>
        Joint 6: {5}</h1>'''.format(
            angles[0], angles[1], angles[2], angles[3],  angles[4], angles[5]
        )
    except:
        return '''<h1>Unable to get angles, try again</h1>'''