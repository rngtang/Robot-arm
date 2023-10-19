from flask import Blueprint
import sys

# for ROS (self-made controls)
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

getCoordinates = Blueprint('getCoordinates', __name__)

@getCoordinates.route("/")
def getCoordinates():
    try:
        coordinates = controls.get_coords()
        return '''<h1>x: {0}\ny: {1}\nz: {2}\nrx: {3}\nry: {4}\nrz: {5}</h1>'''.format(
            coordinates[0], coordinates[1], coordinates[2], coordinates[3],  coordinates[4], coordinates[5],coordinates[6]
        ) 
    except:
        return '''<h1>Unable to get coordinates, try again</h1>'''