# for the robot
from flask import Flask, request

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
    return '''<h1>The given position is: {}, {}</h1>'''.format(coords[0], coords[1])