from flask import Flask, request
import time
import sys

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls

app = Flask(__name__)
controls = Controls()

@app.route("/")
def hello_world():
    return "<p>Hello !!! </p>"

@app.route("/default")
def default_pos: 
     controls.send_angles([0, 0, 0, 0, 0, 0], 70)

if __name__ == '__main__':
    app.run(host='10.194.72.227', port=5001, debug=False)