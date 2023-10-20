# for flask app
from flask import Flask
import sys
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')

# Imports the other functions
from danai_twerk import dance
import default
# from getAngles import getAngles
# from getCoordinates import getCoordinates
from lights import lights
from lightshow import show
# from sendAngles import sendAngles
# from sendCoordinates import sendCoordinates

app = Flask(__name__)

# Default route
@app.route("/")
def hello_world():
    return "<p>Hello !!! </p>"

# Register the blueprints
# app.register_blueprint(default, url_prefix='/default')
app.register_blueprint(lights, url_prefix='/lights')
app.register_blueprint(show, url_prefix='/show')
app.register_blueprint(dance, url_prefix='/dance')
# app.register_blueprint(getAngles, url_prefix='/getAngles')
# app.register_blueprint(getCoordinates, url_prefix='/getCoordinates')
# app.register_blueprint(sendAngles, url_prefix='sendAngles')
# app.register_blueprint(sendCoordinates, url_prefix='sendCoordinates')

if __name__ == '__main__':
    app.run(host='10.194.72.227', port=5000, debug=False)