from flask import Blueprint, current_app
import sys

g_angles = Blueprint('getAngles', __name__)

@g_angles.route("/")
def getAngles():
    controls = current_app.config['controls']
    # Tries to catch any errors
    try:
        angles = controls.get_angles()
        # return '''<h1>Joint 1: {0}\nJoint 2: {1}\nJoint 3: {2}\nJoint 4: {3}\nJoint 5: {4}\nJoint 6: {5}</h1>'''.format(
        #     angles[0], angles[1], angles[2], angles[3],  angles[4], angles[5], angles[6]
        # )
        return 'sdf'
    except:
        return '''<h1>Unable to get angles, try again</h1>'''