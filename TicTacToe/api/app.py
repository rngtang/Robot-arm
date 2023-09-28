# for the robot
from flask import Flask, request
# from pymycobot.mycobot import MyCobot

app = Flask(__name__)
# if __name__ == '__main__':
#     app.run(host='0.0.0.0',port=5000,debug=True)

@app.route("/")
def hello_world():
    return "<p>Hello, World! \nDefault Message</p>"

# position is two numbers (row, col) put together
@app.route("/move")
def access_position():                     
    position = request.args.get('pos')
    # Converts string into a list
    move = [int(position[0]), int(position[1])]
    result = robot_move(position)
    return result

def robot_move(position):
    # return "hi zhichen"
    return '''<h1>The given position is: {}, {}</h1>'''.format(position[0], position[1])