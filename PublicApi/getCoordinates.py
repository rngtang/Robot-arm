from flask import Blueprint, current_app
import sys

getCoordinates = Blueprint('getCoordinates', __name__)

@getCoordinates.route("/")
def getCoordinates():
    controls = current_app.config['controls']
    try:
        coordinates = controls.get_coords()
        return '''<h1>x: {0}\ny: {1}\nz: {2}\nrx: {3}\nry: {4}\nrz: {5}</h1>'''.format(
            coordinates[0], coordinates[1], coordinates[2], coordinates[3],  coordinates[4], coordinates[5],coordinates[6]
        ) 
    except:
        return '''<h1>Unable to get coordinates, try again</h1>'''