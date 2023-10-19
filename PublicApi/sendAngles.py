from flask import request, jsonify, Blueprint
import sys

# for ROS (self-made controls)
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

sendAngles = Blueprint('sendAngles', __name__)

@sendAngles.route("/", methods=['POST'])
def sendAngles():
    # Checks if the user is sending in JSON format
    if not request.is_json():
        return jsonify({"success": False,
                        "message": "Parameter is not in JSON format"
        })
    
    # Gets angles and speed from the parameters
    angles = request.json.get('angles')
    speed = request.json.get('speed')

    # Checks if the parameters are in the correct format and within range
    if len(angles) != 6 or not speed or speed < 0 or speed > 100:
        return jsonify({"success": False,
                        "message": "Invalid parameters"})
    
    # Tries to catch errors
    try:
        controls.send_angles(angles, speed)
        return jsonify({"success": True,
                        "message": "Angles sent successfully"})
    except:
        return jsonify({"success": False,
                        "message": "Failed to send angles"})