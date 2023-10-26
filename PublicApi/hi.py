from flask import jsonify, Blueprint, current_app
import time

# Creates blueprint
hello = Blueprint('hi', __name__)

@hello.route("/")
def hi():
    # Gets the controls object from the app instance
    controls = current_app.config['controls']
    try:
        # Goes to default position
        controls.send_angles([0, 90, -90, 0, 0, 0], 70)
        time.sleep(2)
        controls.send_angles([0, 90, -90, -60, 0, 0], 60)
        time.sleep(2)
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
