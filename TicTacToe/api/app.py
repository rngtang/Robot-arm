# for the robot
from flask import Flask, request

app = Flask(__name__)

@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

# GET endpoint for robot to get where it should move to
@app.route("/move")
def move():
    row = request.args.get('row')
    col = request.args.get('col')
    return '''<h1>The given position is: {}, {}</h1>'''.format(row, col)