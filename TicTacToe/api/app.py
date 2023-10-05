# for the robot
from flask import Flask, request
# import requests
import time
from pymycobot.mycobot import MyCobot
# mc = MyCobot("/dev/ttyAMA0", 1000000)

app = Flask(__name__)

import sys
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')

from controls import Controls

controls = Controls()

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
    print(coords)
    result = robot_move(coords)
    return result

def robot_move(coords):

    # Johnny's
    # Sends the position coordinates to the Flask API
    # url = "http://localhost:5000/move"
    # params = {"pos": str(coords[0]) + str(coords[1])}
    # response = requests.get(url, params=params)

    # Returns the response from the Flask API
    # return response.text

    # Raul's
    # Testing movements
    # if(coords[0] == 1):
    #     mc.sync_send_angles([0, -135, 90, -50, 0, 0], 60, 3)
    # elif(coords[0] == 2):
    #     mc.sync_send_angles([0, -135, 90, -50, 0, 0], 60, 3)
    # time.sleep(3)
    # mc.send_angles([0, 0, 0, 0, 0, 0], 70)
    # # Prints the coordinates to move

    if coords[0] == coords[1] and coords[0] == 0:
        controls.send_coords([249, 32, -12, 180, 1, -48], 70, 2)
        time.sleep(2)
        controls.send_angles([0, 0, 0, 0, 0, 0], 70)

    elif coords[0] == coords[1] and coords[0] == 1:
        controls.send_coords([239, 4, -12, -177, 1, -54], 70, 2)
        time.sleep(2)
        controls.send_angles([0, 0, 0, 0, 0, 0], 70)

    else:
        controls.send_coords([221, -23, -12, -180, 0, -59], 70, 2)
        time.sleep(2)
        controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    return '''<h1>The given position is: {}, {}</h1>'''.format(coords[0], coords[1])

if __name__ == '__main__':
    app.run(host='0.0.0.0',port=5000,debug=True)