from flask import request, jsonify, Blueprint
import sys

# for ROS (self-made controls)
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

sendCoordinates = Blueprint('sendCoordinates', __name__)

@sendCoordinates.route("/", methods=['POST'])
def sendCoordinates():
    # Checks if the user is sending in JSON format
    if not request.is_json():
        return jsonify({"success": False,
                        "message": "Parameter is not in JSON format"
        })
    
    # Gets angles and speed from the parameters
    coordinates = request.json.get('coords')
    speed = request.json.get('speed')
    mode =  request.json.get('mode')

    # Checks if the parameters are in the correct format and within range
    if len(coordinates) != 6 or not speed or speed < 0 or speed > 100 or not mode or (mode != 0 and mode != 1):
        return jsonify({"success": False,
                        "message": "Invalid parameters"})
    
    # Tries to catch errors
    try:
        controls.send_coords(coordinates, speed, mode)
        return jsonify({"success": True,
                        "message": "Coordinates sent successfully"})
    except:
        return jsonify({"success": False,
                        "message": "Failed to send coordinates"})