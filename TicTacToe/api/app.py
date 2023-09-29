# for the robot
from flask import Flask, request
from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyAMA0", 1000000)

app = Flask(__name__)
# if __name__ == '__main__':
#     app.run(host='0.0.0.0',port=5000,debug=True)

@app.route("/")
def hello_world():
    return "<p>Hello, World! \nDefault Message</p>"

# position is two numbers (row, col) put together
@app.route("/move")
def access_position():
    # Gets the coordinates from the arguments  
    position = request.args.get('pos')

    # Converts string into a list
    coords = [int(position[0]), int(position[1])]
    result = robot_move(coords)
    return result

def robot_move(coords):
    # Prints the coordinates to move
    if(coords[0] == 1):
        mc.send_coords([160, 160, 160, 0, 0, 0], 70, 0)
        print(mc.get_coords())
    elif(coords[0] == 2):
        mc.send_angles([90, 90, -90, 0, 0, 0], 60)
    return '''<h1>The given position is: {}, {}</h1>'''.format(coords[0], coords[1])