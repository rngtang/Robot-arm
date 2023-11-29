from flask import Blueprint, current_app

# Creates the blueprint
release = Blueprint('release', __name__)

@release.route("/")
def releaseJoints():
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
        # # Tries to catch any errors
        try: 
            # controls.toggle_servo_released(True)
            mc.release_all_servos()
            return '''<h1>released </h1>''', 200
        except:
            return '''<h1>Unable to get release joints, try again</h1>''', 400
   