from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
from pymycobot import PI_PORT, PI_BAUD
import time
import datetime

print("Start running")
mc = MyCobot(PI_PORT, PI_BAUD)

time.sleep(3)
print("Connected")
print(f"Initial coords: {mc.get_coords()}")

# set start start time
start = time.time()
print("Starting")
# Let the robotic arm reach the specified position
mc.send_angles([-1.49, 115, -153.45, 30, -33.42, 137.9], 80)
# Determine if it reaches the specified position
print("Entering loop")
while time.time() - start <= 10:
    print("Looping")
    mc.get_coords()
    time.sleep(0.5)
    # mc.get_angles()

    mc.release_all_servos()

# while not mc.is_in_position([-1.49, 115, -153.45, 30, -33.42, 137.9], 0):
#     # Return the robotic arm to motion
#     mc.resume()
#     # Let the robotic arm move for 0.5s
#     time.sleep(0.5)
#     # Pause arm movement
#     mc.pause()
#     # Determine if the move timed out
#     if time.time() - start > 3:
#         break

# # set start time
# start = time.time()
# # Let the exercise last for 30 seconds
# while time.time() - start < 30:
#     # Let the robotic arm quickly reach this position
#     mc.send_angles([-1.49, 115, -153.45, 30, -33.42, 137.9], 80)
#     # Set the color of the light to [0,0,50]
#     mc.set_color(0, 0, 50)
#     time.sleep(0.7)
#     # Let the robotic arm quickly reach this position
#     mc.send_angles([-1.49, 55, -153.45, 80, 33.42, 137.9], 80)
#     # Set the color of the light to [0,50,0]
#     mc.set_color(0, 50, 0)
#     time.sleep(0.7)
