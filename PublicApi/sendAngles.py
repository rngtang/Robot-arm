from flask import request, jsonify, Blueprint, current_app

# Creates blueprint
s_angles = Blueprint('sendAngles', __name__)

@s_angles.route("/", methods=['POST'])
def sendAngles():
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
        angles = request.json.get('angles')
        speed = request.json.get('speed')

        # Checks if the parameters are in the correct format and within range
        if len(angles) != 6 or (speed < 0 or speed > 100):
            return jsonify({"success": False,
                            "message": "Invalid parameters"}), 400
        
        for value in angles:
            if value < -165 or value > 165:
                return jsonify({"success": False,
                            "message": "Invalid parameters"}), 400
            
        # Tries to catch errors
        try:
            # controls.send_angles(angles, speed)
            mc.send_angles(angles, speed)
            return jsonify({"success": True,
                            "message": "Angles sent successfully"}), 200
        except:
            return jsonify({"success": False,
                            "message": "Failed to send angles"}), 500