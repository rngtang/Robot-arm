from flask import Flask, request
import time
import sys

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls

app = Flask(__name__)
controls = Controls()