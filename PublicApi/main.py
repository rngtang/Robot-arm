# for flask app
from flask import Flask, request, jsonify
import time
import sys

# Imports the other functions
from danai_twerk import dance
from default import default
from getAngles import getAngles
from getCoordinates import getCoordinates
from lights import lights
from lightshow import lightshow
from sendAngles import sendAngles
from sendCoordinates import sendCoordinates

# for lights and dance (asynch)
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
mc = MyCobot(PI_PORT, PI_BAUD)

# for ROS (self-made controls)
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
app = Flask(__name__)
controls = Controls()

# Default route
@app.route("/")
def hello_world():
    return "<p>Hello !!! </p>"

# Register the blueprints
app.register_blueprint(default, url_prefix='/default')


@app.route("/lights") 
def change_lights(): # given parameters
    red = request.args.get('r', 10)
    green = request.args.get('g', 10)
    blue = request.args.get('b', 10)

    mc.set_color(int(red),int(green),int(blue))
    print(red)
    print(green)
    print(blue)
    # return '''<h1>what is GOING ON</h1>'''
    time.sleep(1)
    return'''<h1>The given color is: {}, {}, {}</h1>'''.format(red, green, blue)


@app.route("/lightshow")
def light_gradient(): # given no parameters
    for count in range(0,3):
        #From red (255,0,0) to blue (0,0,255)
        for i in range(0,255):
            red = 255 - i
            green = 0
            blue = i
            mc.set_color(red,green,blue)
            time.sleep(0.001)

        #From blue (0,0,255) to green (0,255,0)
        for i in range(0,255):
            red = 0
            green = i
            blue = 255 - i
            mc.set_color(red,green,blue)
            time.sleep(0.001)   

        #From green (0,255,0) to red (255,0,0)
        for i in range(0,255):
            red = i
            green = 255 - i
            blue = 0
            mc.set_color(red,green,blue)
            time.sleep(0.001)   

    return '''<h1>red, blue, green</h1>'''


@app.route("/dance")
def danai_twerk():
    # set start start time
    start = time.time()
    # Let the robotic arm reach the specified position
    mc.send_angles([-1.49, 115, -153.45, 30, -33.42, 137.9], 80)
    # Determine if it reaches the specified position
    while not mc.is_in_position([-1.49, 115, -153.45, 30, -33.42, 137.9], 0):
        # Return the robotic arm to motion
        mc.resume()
        # Let the robotic arm move for 0.5s
        time.sleep(0.5)
        # Pause arm movement
        mc.pause()
        # Determine if the move timed out
        if time.time() - start > 3:
            break

    # set start time
    start = time.time()
    # Let the exercise last for 15 seconds
    while time.time() - start < 15:
        # Let the robotic arm quickly reach this position
        mc.send_angles([-1.49, 115, -153.45, 30, -33.42, 137.9], 80)
        # Set the color of the light to [0,0,50]
        mc.set_color(0, 0, 50)
        time.sleep(0.7)
        # Let the robotic arm quickly reach this position
        mc.send_angles([-1.49, 55, -153.45, 80, 33.42, 137.9], 80)
        # Set the color of the light to [0,50,0]
        mc.set_color(0, 50, 0)
        time.sleep(0.7)
    return '''<h1>gonna go crazy </h1>'''

# Returns the current angles of the robot
@app.route("/getAngles")
def getAngles():
    # Tries to catch any errors
    try:
        angles = controls.get_angles()
        return '''<h1>Joint 1: {0}\nJoint 2: {1}\nJoint 3: {2}\nJoint 4: {3}\nJoint 5: {4}\nJoint 6: {5}</h1>'''.format(
            angles[0], angles[1], angles[2], angles[3],  angles[4], angles[5], angles[6]
        )
    except:
        return '''<h1>Unable to get angles, try again</h1>'''

# Returns the current coordinates of the robot
@app.route("/getCoordinates")
def getCoordinates():
    try:
        coordinates = controls.get_coords()
        return '''<h1>x: {0}\ny: {1}\nz: {2}\nrx: {3}\nry: {4}\nrz: {5}</h1>'''.format(
            coordinates[0], coordinates[1], coordinates[2], coordinates[3],  coordinates[4], coordinates[5],coordinates[6]
        ) 
    except:
        return '''<h1>Unable to get coordinates, try again</h1>'''

# Sends angles to the robot
@app.route("/sendAngles", methods=['POST'])
def sendAngles():
    # Checks if the user is sending in JSON format
    if not request.is_json():
        return jsonify({"success": False,
                        "message": "Parameter is not in JSON format"
        })
    
    # Gets angles and speed from the parameters
    angles = request.json.get('angles')
    speed = request.json.get('speed')

    # Checks if the parameters are in the correct format and within range
    if len(angles) != 6 or not speed or speed < 0 or speed > 100:
        return jsonify({"success": False,
                        "message": "Invalid parameters"})
    
    # Tries to catch errors
    try:
        controls.send_angles(angles, speed)
        return jsonify({"success": True,
                        "message": "Angles sent successfully"})
    except:
        return jsonify({"success": False,
                        "message": "Failed to send angles"})
    
# Send coordinates
@app.route("/sendCoordinates", methods=['POST'])
def sendCoordinates():
    # Checks if the user is sending in JSON format
    if not request.is_json():
        return jsonify({"success": False,
                        "message": "Parameter is not in JSON format"
        })
    
    # Gets angles and speed from the parameters
    coordinates = request.json.get('coords')
    speed = request.json.get('speed')
    mode =  request.json.get('mode')

    # Checks if the parameters are in the correct format and within range
    if len(coordinates) != 6 or not speed or speed < 0 or speed > 100 or not mode or (mode != 0 and mode != 1):
        return jsonify({"success": False,
                        "message": "Invalid parameters"})
    
    # Tries to catch errors
    try:
        controls.send_coords(coordinates, speed, mode)
        return jsonify({"success": True,
                        "message": "Coordinates sent successfully"})
    except:
        return jsonify({"success": False,
                        "message": "Failed to send coordinates"})

if __name__ == '__main__':
    app.run(host='10.194.72.227', port=5001, debug=False)