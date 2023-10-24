from flask import request, jsonify, Blueprint, current_app

s_coordinates = Blueprint('sendCoordinates', __name__)

@s_coordinates.route("/", methods=['POST'])
def sendCoordinates():
    controls = current_app.config['controls']
    # Checks if the user is sending in JSON format
    if not request.is_json:
        return jsonify({
            "success": False,
            "message": "Parameter is not in JSON format"
        }), 400
    
    # Gets angles and speed from the parameters
    coordinates = request.json.get('coords')
    speed = request.json.get('speed')
    mode = request.json.get('mode')

    print(mode)

    # Checks if the parameters are in the correct format and within range
    if len(coordinates) != 6 or not speed or (speed < 0 or speed > 100) or not mode or not(0<=mode<=1):
        return jsonify({"success": False,
                        "message": "Invalid parameters"}), 400
    
    # Tries to catch errors
    try:
        controls.send_coords(coordinates, speed, mode)
        return jsonify({"success": True,
                        "message": "Coordinates sent successfully"})
    except:
        return jsonify({"success": False,
                        "message": "Failed to send coordinates"}), 400