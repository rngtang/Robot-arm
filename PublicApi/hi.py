from flask import jsonify, Blueprint, current_app
import time

# Creates blueprint
hello = Blueprint('hi', __name__)

@hello.route("/")
def hi():
    lock = current_app.config['lock']
    if lock.locked():
        return "A request is already in progress"
    
    with lock:
        # Gets the controls object from the app instance
        try: 
            controls = current_app.config['controls']
        except: 
            return '''<h1>Unable to connect to Controls</h1>''', 500
        
        try:
            # Goes to default position
            controls.send_angles([0, 90, -90, 0, 0, 0], 70)
            time.sleep(2)
            controls.send_angles([0, 90, -90, -60, 0, 0], 60)
            time.sleep(1)
            controls.send_angles([0, 90, -90, 0, 0, 0], 60)

            return jsonify({
                "success": True,
                "message": "hi"
            }), 200
        except:
            return jsonify({
                "success": False,
                "message": "The robot doesn't like you"
            }), 400
