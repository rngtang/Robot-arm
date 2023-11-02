# for flask app
from flask import Flask
import sys

# # judy testing : https://github.com/RobotWebTools/rosbridge_suite
# import os
# import rospy
# import threading

# from std_msgs.msg import UInt32

# def ros_callback(msg):
#     print(msg)

# threading.Thread(target=lambda: rospy.init_node('controls', disable_signals=True)).start()
# rospy.Subscriber('/listener', UInt32, ros_callback)
# pub = rospy.Publisher('/talker', UInt32, queue_size=10)
############################

# Imports the other functions
from danai_twerk import dance
from getAngles import g_angles
from getCoordinates import g_coordinates
from default import default
from lights import lights
from lightshow import show
from sendAngles import s_angles
from sendCoordinates import s_coordinates
from release import release
from hi import hello

# Creates the flask app instance
app = Flask(__name__)

print("FROM MAIN: APP RUNNING")

try: 
    sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
except:
    print("FROM MAIN: cannot sys.path.append")

try: 
    # imports the ROS package and attaches the controls object to the Flask instance
    from controls import Controls
except: 
    print("FROM MAIN: cannot import Controls")

if "controls" not in sys.modules: 
    print("FROM MAIN: do not have controls in sys.modules")

print("FROM MAIN: right before ")
try: 
    c = Controls()
except Exception as error:
    # handle the exception
    print("FROM MAIN: An exception occurred:", error) # An exception occurred: division by zero

try: 
    app.config['controls'] = c
except: 
    print("FROM MAIN: cannot connect to Controls")

# Creates MyCobot object and attaches it to the Flask instance
from pymycobot.mycobot import MyCobot
app.config['mc'] = MyCobot("/dev/ttyAMA0", 1000000)

# Default route (home page)
@app.route("/")
def hello_world():
    return "<p>Hello !!! </p>"

# Register the blueprints    
app.register_blueprint(lights, url_prefix='/lights')
app.register_blueprint(show, url_prefix='/show')
app.register_blueprint(dance, url_prefix='/dance')
app.register_blueprint(default, url_prefix='/default')
app.register_blueprint(g_angles, url_prefix='/getAngles')
app.register_blueprint(g_coordinates, url_prefix='/getCoordinates')
app.register_blueprint(s_angles, url_prefix='/sendAngles')
app.register_blueprint(s_coordinates, url_prefix='/sendCoordinates')
app.register_blueprint(release, url_prefix='/release')
app.register_blueprint(hello, url_prefix='/hi')

if __name__ == '__main__':
    # app.run(host='10.194.72.227', port=5000, debug=False)
    # app.run(host=os.environ['ROS_IP'], port=5000, debug=False)

    # new IP
    app.run(host='10.194.29.175', port=5000, debug=False)

