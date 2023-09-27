# for the robot
from flask import Flask, request

app = Flask(__name__)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

# GET endpoint for robot to get where it should move to
@app.route('/move/<position>', methods=['GET'])
@app.route('/move/', defaults={'position' : '99'})
def move(position):
    row = position[0]
    col = position[1]
    return '''<h1>The given position is: {}, {}</h1>'''.format(row, col)