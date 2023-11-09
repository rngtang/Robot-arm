from flask import Blueprint, current_app

# Creates the blueprint
g_coordinates = Blueprint('getCoordinates', __name__)

@g_coordinates.route("/")
def getCoordinates():
    # Gets the controls object from the app instance
    try: 
        controls = current_app.config['controls']
    except: 
        return '''<h1>Unable to connect to Controls</h1>''', 500
    # Tries to catch any errors
    try:
        coordinates = controls.get_coords()
        return '''<h1>x: {0}<br>
        y: {1}<br>
        z: {2}<br>
        rx: {3}<br>
        ry: {4}<br>
        rz: {5}</h1>'''.format(
            coordinates[0], coordinates[1], coordinates[2], coordinates[3],  coordinates[4], coordinates[5]
        ), 200
    except:
        return '''<h1>Unable to get coordinates, try again</h1>''', 400