# for the robot
from flask import Flask, request
# import requests
import time
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

app = Flask(__name__)

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
    # Prints the coordinates to move
    return '''<h1>The given position is: {}, {}</h1>'''.format(coords[0], coords[1])


if __name__ == '__main__':
    app.run(host='0.0.0.0',port=5001,debug=True)