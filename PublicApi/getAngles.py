from flask import Blueprint, current_app

# Creates blueprint
g_angles = Blueprint('getAngles', __name__)

@g_angles.route("/")
def getAngles():
    lock = current_app.config['lock']
    if lock.locked():
        return "A request is already in progress"

    with lock:
        # Gets the controls object from the app instance
        try: 
            controls = current_app.config['controls']
        except: 
            return '''<h1>Unable to connect to Controls</h1>''', 500
        # Tries to catch any errors
        try:
            angles = controls.get_angles()
            return '''<h1>Joint 1: {0}<br>
            Joint 2: {1}<br>
            Joint 3: {2}<br>
            Joint 4: {3}<br>
            Joint 5: {4}<br>
            Joint 6: {5}</h1>'''.format(
                angles[0], angles[1], angles[2], angles[3],  angles[4], angles[5]
            ), 200
        except:
            return '''<h1>Unable to get angles, try again</h1>''', 400