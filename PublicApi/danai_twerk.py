from flask import Blueprint, current_app
import time

dance = Blueprint('dance', __name__)

@dance.route("/")
def danai_twerk():
    # Gets the myCobot object from the app instance
    mc = current_app.config['mc']
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
        mc.resume()
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
    return '''<h1>gonna go crazy </h1>''', 200