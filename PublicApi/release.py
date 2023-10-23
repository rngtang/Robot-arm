from flask import Blueprint, current_app

# Creates the blueprint
release = Blueprint('release', __name__)

@release.route("/")
def releaseJoints():
    controls = current_app.config['controls']
    try: 
        controls.toggle_servo_released(True)
        return '''<h1>release </h1>'''
    except:
        return '''<h1>Unable to get release joints, try again</h1>'''
   