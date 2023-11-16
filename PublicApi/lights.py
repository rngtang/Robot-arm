from flask import request, Blueprint, current_app
import time

# testing -> why can't it be called as a config ? 
from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

# Creates the blueprint
lights = Blueprint('lights', __name__)

@lights.route("/")
def change_lights(): # given parameters
    lock = current_app.config['lock']
    if lock.locked():
        return "A request is already in progress"
    
    with lock:
        # Gets the myCobot object from the app instance
        # mc = current_app.config['mc']
        red = request.args.get('r', 10)
        green = request.args.get('g', 10)
        blue = request.args.get('b', 10)

        mc.set_color(int(red),int(green),int(blue))
        print(red)
        print(green)
        print(blue)
        # return '''<h1>what is GOING ON</h1>'''
        time.sleep(1)
        return'''<h1>The given color is: {}, {}, {}</h1>'''.format(red, green, blue), 200