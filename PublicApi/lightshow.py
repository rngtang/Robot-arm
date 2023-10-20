from flask import Blueprint
import time

from pymycobot.mycobot import MyCobot
mc = MyCobot("/dev/ttyAMA0", 1000000)

show = Blueprint('show', __name__)

@show.route("/")
def lightshow(): # given no parameters
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