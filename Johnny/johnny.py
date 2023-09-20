from pymycobot.mycobot import MyCobot

from pymycobot.genre import Angle

from pymycobot import PI_PORT, PI_BAUD

from pymycobot.mypalletizer import MyPalletizer

from pymycobot.genre import Coord

import time

import random

# mc.send_angle(Angle.J2.value, 0, 100)

# mc = MyCobot("COM3", 115200)

# i = 7
# #loop 7 times
# while i > 0:                            
#     mc.set_color(0,0,255) #blue light on
#     time.sleep(2)    #wait for 2 seconds                
#     mc.set_color(255,0,0) #red light on
#     time.sleep(2)    #wait for 2 seconds
#     mc.set_color(0,255,0) #green light on
#     time.sleep(2)    #wait for 2 seconds
#     i -= 1

#     mc = MyCobot(PI_PORT, PI_BAUD)

# Check whether the program can be burned into the robot arm

# Fine-tune the robotic arm to ensure that all the bayonets are aligned in the adjusted position
# Subject to the alignment of the mechanical arm bayonet, this is only a case

#reset angles to 0 
# mc.send_angles([0, 0, 0, 0, 0, 0], 30)

# Robotic arm recovery
# mc.send_angles([0, 0, 0, 0, 0, 0], 40)
# time.sleep(3)

# mc.send_angles([0, 0, 0, 0, 0, 0], 30)

mc = MyCobot("/dev/ttyAMA0", 1000000)

mc.send_angles([0, 0, 0, 0, 0, 0], 30)
time.sleep(3)

# mc.send_angles([0, 90, 20, 0, 0, 0], 30)

time.sleep(3)
mc.set_color(0,0,255) #blue light on

# Magic eightBall
# for i in range(1):
#     ran = random.randint(0, 3)
#     time.sleep(2)
#     if ran == 0:
#         print("test1")
#         mc.set_color(255,0,0) #red light on
#         mc.send_angles([90, 0, 0, 0, 0, 0], 100)
#         time.sleep(0.5)
#         mc.send_angles([90, 0, 0, 0, -35, 0], 100)
#         time.sleep(0.5)
#         mc.send_angles([90, 0, 0, 0, 35, 0], 100)
#         time.sleep(1)
#         mc.send_angles([90, 0, 0, 0, 0, 0], 100)
#         time.sleep(1)
#         print("test2")
#     if ran == 1:
#         print("test3")
#         mc.set_color(0,255,0) #green light on
#         mc.send_angles([90, 0, 0, 0, 0, 0], 100)
#         time.sleep(0.5)
#         mc.send_angles([90, 0, 0, 30, 0, 0], 100)
#         time.sleep(0.5)
#         mc.send_angles([90, 0, 0, 0, 0, 0], 100)
#         time.sleep(1)
#         print("test4")
#     if ran == 2:
#         print("test3")
#         mc.set_color(255,255,0) #yellow light on
#         mc.send_angles([0, -35, 60, -135, 90, 0], 100)
#         time.sleep(1)
#         print("test4")



# Initiate a MyPalletizer object
# mc = MyPalletizer("COM3", 115200)

# Get the current coordinates and pose of the head
coords = mc.get_coords()
print(coords)

# mc.send_coords([-91.8, -27.9, 159.4, 89.98, 3.51, -172.0], 30, 0) #spell n


# mc.send_coords([38.6, -107.4, 276.7, 86.44, 69.83, -1.07], 30, 0) #spell J

mc.send_coords([-57.3, -39.1, 145.1, -80.64, 25.52, 8.51], 30, 0) #spell D

# [37.7, -107.5, 276.8, 86.22, 69.65, -1.28] j
# [38.6, -107.4, 276.7, 86.44, 69.83, -1.07] j
# [-57.3, -39.1, 145.1, -80.64, 25.52, 8.51] d
# [-57.4, -38.5, 145.6, -80.06, 25.51, 8.76] d
# mc.send_coords([-51.3, 48.3, 161.6, 161.39, -5.87, -70.31], 30, 0) #spell D




# mc.send_coords([280.3, -65.3, 82.2, 179.21, 0.26, -90.17], 30, 0)
# time.sleep(3)
# mc.send_coords([61.4, -283.3, 167.5, 2.11, -0.25, -0.62], 30, 0)
# time.sleep(3)
# mc.send_coords([-283.1, -63.1, 167.2, 2.28, 0.27, -90.34], 30, 0)
# time.sleep(3)
# mc.send_coords([-65.7, 282.3, 167.5, 2.11, -0.25, -179.74], 30, 0)
# time.sleep(3)
# mc.send_coords([43.5, -63.1, 412.9, -89.64, -0.26, -89.65], 30, 0)



# Plan the route at random, let the head reach the coordinates of [57.0, -107.4, 316.3] in an non-linear manner at the speed is 80mm/s
# mc.send_coords([-283.2, -62.7, 169.4, 2.1, 0.27, -90.42], 30, 0)
# wait for 2 seconds
# time.sleep(2)

# Plan the route at random, let the head reach the coordinates of [207.9, 47, 49.3,-159.69] in an non-linear manner at the speed is 80mm/s
# mc.send_coords([207.9, 47, 49.3,-159.69], 30, 0)
# wait for 2 seconds
# time.sleep(2)

# To change only the x-coordinate of the head, set the x-coordinate of the head to 20. Let it plan the route at random and move the head to the changed position at a speed of 70mm/s
# mc.send_coord(Coord.X.value, 30, 10)