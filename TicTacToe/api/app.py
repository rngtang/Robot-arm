from flask import Flask, request
import time
import sys

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls

# Creates the controls object and the instance of the flask app 
app = Flask(__name__)
controls = Controls()

# start in default position (up)
time.sleep(2)
controls.send_angles([0, 0, 0, 0, 0, 0], 70)
print("Default position")
time.sleep(2)

# Angles dictionary
TIC_TAC_TOE_ANGLES = {
    "00": [22.57, -71.44, -76.01, 55.88, -3.95, -11.06],
    "10": [25.21, -73.3, -88.15, 79.62, -7.1, -11.06],
    "20": [27.42, -70.83, -98.16, 86.04, -10.1, -11.06],
    "01": [16.87, -73.3, -80.23, 67.58, -1.3, -11.15],
    "11": [18.45, -70.30, -88.94, 72.5, -1, -11.06],
    "21": [18.79, -69.69, -96.84, 81.29, -3.59, -11.6],
    "02": [11.15, -71.8, -73.81, 51.31, -2.80, -11.25],
    "12": [12.11, -70.48, -85.59, 66.34, -.13, 11.06],
    "22": [11.85, -65.3, -93.25, 64.33, 1.65, -11.15]
}


# Default route
@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

# Gets the request and validates it
@app.route("/move")
def access_position():
    # Gets the coordinates from the arguments
    position = request.args.get('pos')

    # Fail safe?
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)

    print(coords)
    # Checks if coords are valid
    if coords not in TIC_TAC_TOE_ANGLES:
        return "<h1>This is not a valid coordinate, please try again</h1>"

    # Converts string into a list
    coords = [int(position[0]), int(position[1])]
    time.sleep(1)
    print("SEND")
    result = robot_move(coords)
    time.sleep(2)
    print("RETURN")
    return result

# Moves the robot
def robot_move(coords):
    # Tries catching any errors when sending the coordinates
    try:
        controls.send_angles(TIC_TAC_TOE_ANGLES[coords], 70)
        # will print "Angles Published" <- every time controls.send_angles() method is called
        time.sleep(1)
        controls.send_angles([0, 0, 0, 0, 0, 0], 70)
        time.sleep(1)
        return
    except:
        print("FROM APP: cannot call on ROS")

if __name__ == '__main__':
    try:
        app.run(host='10.194.72.227', port=5000, debug=False)
    except Exception:
        print("FROM APP: failed to run app.run LOL")
