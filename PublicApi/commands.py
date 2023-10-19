# for flask app
from flask import Flask, request
import time
import sys

# for lights and dance (asynch)
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
mc = MyCobot(PI_PORT, PI_BAUD)

# for ROS (self-made controls)
sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
app = Flask(__name__)
controls = Controls()

# routes
@app.route("/")
def hello_world():
    return "<p>Hello !!! </p>"


# move to upright position
@app.route("/default")
def default_pos(): 
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    mc.set_color(255, 255, 255) # start white
    time.sleep(1)
    return '''<h1>default </h1>'''


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


if __name__ == '__main__':
    app.run(host='10.194.72.227', port=5001, debug=False)