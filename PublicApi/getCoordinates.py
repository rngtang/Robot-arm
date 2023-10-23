from flask import Blueprint, current_app

g_coordinates = Blueprint('getCoordinates', __name__)

@g_coordinates.route("/")
def getCoordinates():
    controls = current_app.config['controls']
    try:
        coordinates = controls.get_coords()
        return '''<h1>x: {0}<br>
        y: {1}<br>
        z: {2}<br>
        rx: {3}<br>
        ry: {4}<br>
        rz: {5}</h1>'''.format(
            coordinates[0], coordinates[1], coordinates[2], coordinates[3],  coordinates[4], coordinates[5]
        ) 
    except:
        return '''<h1>Unable to get coordinates, try again</h1>'''