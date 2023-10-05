from flask import Flask, request
import time

app = Flask(__name__)

import sys
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')

from controls import Controls

controls = Controls()

TIC_TAC_TOE_COORDS = {
    "00": [248, 30, -12],
    "10": [235, 28, -13],
    "20": [221, 30, -13],
    "01": [248, 4, -13],
    "11": [235, 2, -13],
    "21": [222, -2, -12],
    "02": [249, -23, -11],
    "12": [235, -23, -12],
    "22": [220, -23, -14]
}

# Default route
@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

# Route to move the robot
@app.route("/move")
def access_position():
    # Gets the coordinates from the arguments  
    position = request.args.get('pos')

    # Converts string into a list
    coords = [int(position[0]), int(position[1])]
    result = robot_move(coords)
    return result

def robot_move(coords):
    if coords not in TIC_TAC_TOE_COORDS:
        return "<h1>This is not a valid coordinate, please try again</h1>"

    controls.send_coords(TIC_TAC_TOE_COORDS[coords].extend([180, 0, 0]), 70, 2)
    time.sleep(2)
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)

    return '''<h1>The given position is: {}, {}</h1>'''.format(coords[0], coords[1])

if __name__ == '__main__':
    app.run(host='0.0.0.0',port=5000,debug=True)