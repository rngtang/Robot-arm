from flask import request, jsonify, Blueprint, current_app

# Creates the blueprint
s_coordinates = Blueprint('sendCoordinates', __name__)

@s_coordinates.route("/", methods=['POST'])
def sendCoordinates():
    lock = current_app.config['lock']
    mc = current_app.config['mc']
    if lock.locked():
        return "A request is already in progress"
    
    with lock:
        # # Gets the controls object from the app instance
        # try: 
        #     controls = current_app.config['controls']
        # except: 
        #     return '''<h1>Unable to connect to Controls</h1>''', 500
            
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

        # Checks if the parameters are in the correct format and within range
        if len(coordinates) != 6 or (speed < 0 or speed > 100) or (mode != 0 and mode != 1):
            return jsonify({"success": False,
                            "message": "Invalid parameters"}), 400
        
        # Tries to catch errors
        try:
            # controls.send_coords(coordinates, speed, mode)
            mc.send_coords(coordinates, speed, mode)
            return jsonify({"success": True,
                            "message": "Coordinates sent successfully"}), 200
        except:
            return jsonify({"success": False,
                            "message": "Failed to send coordinates"}), 500