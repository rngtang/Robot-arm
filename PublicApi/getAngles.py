from flask import Blueprint
import sys

# for ROS (self-made controls)
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

getAngles = Blueprint('getAngles', __name__)

@getAngles.route("/")
def getAngles():
    # Tries to catch any errors
    try:
        angles = controls.get_angles()
        return '''<h1>Joint 1: {0}\nJoint 2: {1}\nJoint 3: {2}\nJoint 4: {3}\nJoint 5: {4}\nJoint 6: {5}</h1>'''.format(
            angles[0], angles[1], angles[2], angles[3],  angles[4], angles[5], angles[6]
        )
    except:
        return '''<h1>Unable to get angles, try again</h1>'''