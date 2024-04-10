from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord
import time
import math
import numpy as np

mc = MyCobot("/dev/ttyAMA0", 1000000)

# mc.release_all_servos()

# mc.send_angles([0, 0, 0, 0, 0, 0], 40)
# time.sleep(10)
# coords = mc.get_coords()
# # time.sleep(8)
# print(math.sqrt(coords[0]**2 + coords[1]**2))


# mc.send_angles([0, -90, 0, 0, 0, 0], 40)
# time.sleep(12)
# coords = mc.get_coords()
# time.sleep(8)
# print(math.sqrt(coords[0]**2 + coords[1]**2) - 90)
# while True:

mc.send_angles([0, -60, 60, 0, 0, -135], 20)
time.sleep(2)


coords = mc.get_coords()
time.sleep(3)
# x = coords[0]  # Example x coordinate
# y = coords[1]  # Example y coordinate
# z = coords[2]  # Example z coordinate
# rx = coords[3]  # Example rotation around x axis (in degrees)
# ry = coords[4]  # Example rotation around y axis (in degrees)
# rz = coords[5]  # Example rotation around z axis (in degrees)

# distance = 10


# for i in range(10):
    # angles = mc.get_angles()
    # if len(coords) > 0 and len(angles) > 0:
    # if len(angles) > 0:
    # time.sleep(1)
    # if len(coords) > 0:

        # print(max(0, math.sqrt(coords[0]**2 + coords[1]**2) - 110))
        # print(angles[1],",", angles[2], ",", angles[1] + angles[2])
        # if abs(angles[1]) > 90:

        # j2_angle = angles[1]
        # j3_angle = angles[2]
        # value = -abs(abs(angles[2]) - abs(angles[1]))
        # calculations = abs((-abs(j2_angle) / 90) + 1)
        # if abs(j2_angle) > 35 and abs(j2_angle) < 145:
        #     value = (abs(angles[2]) - abs(angles[1]))
        # calculations += 0.87 * (abs((( value / ( 90)) + 1)))   
        # calculations = min(1.87, calculations)
        # print(calculations)
        # camera_angle = abs(angles[1] + angles[2] + angles[3])
        # print(camera_angle)
        # print(coords[3:])

        # rx_rad, ry_rad, rz_rad = np.radians(rx), np.radians(ry), np.radians(rz)
        # x = x + distance * np.cos(ry_rad) * np.cos(rz_rad)
        # y = y + distance * np.cos(ry_rad) * np.sin(rz_rad)
        # z = z + distance * np.sin(ry_rad)

        # Print the new position
        # print(f"x: {x}, y: {y}, z: {z}")
        # print("iter")
        # mc.send_coords([x, y, z, rx, ry, rz], 10)
        # time.sleep(1)

def translation_matrix(tx, ty, tz):
    """
    Create a translation matrix.

    Args:
        tx (float): Translation along x-axis.
        ty (float): Translation along y-axis.
        tz (float): Translation along z-axis.

    Returns:
        np.ndarray: 4x4 translation matrix.
    """
    return np.array([[1, 0, 0, tx],
                     [0, 1, 0, ty],
                     [0, 0, 1, tz],
                     [0, 0, 0, 1]])

def rotation_matrix(axis, angle):
    """
    Create a rotation matrix for a given axis and angle.

    Args:
        axis (np.ndarray): Rotation axis (3D vector).
        angle (float): Rotation angle in radians.

    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    axis /= np.linalg.norm(axis)
    a = math.cos(angle / 2.0)
    b, c, d = -axis * math.sin(angle / 2.0)
    return np.array([[a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d + a*c)],
                     [2*(b*c + a*d), a*a + c*c - b*b - d*d, 2*(c*d - a*b)],
                     [2*(b*d - a*c), 2*(c*d + a*b), a*a + d*d - b*b - c*c]])

def move_robot(coords, forward=True, distance=1.0):
    """
    Move the robot forward or backward along the angle of the head using coordinate transformations.

    Args:
        coords (list): List of coordinates [x, y, z, rx, ry, rz].
        forward (bool): True for forward movement, False for backward movement.
        distance (float): Distance to move (default is 1.0).

    Returns:
        list: New coordinates after movement.
    """
    # Extract coordinates
    x, y, z, rx, ry, rz = coords

    # Convert angles to radians
    rx_rad, ry_rad, rz_rad = math.radians(rx), math.radians(ry), math.radians(rz)

    # Calculate direction vector based on orientation angles
    dx = math.cos(rx_rad) * math.cos(ry_rad)
    dy = math.sin(rx_rad) * math.cos(ry_rad)
    dz = math.sin(ry_rad)

    # Normalize direction vector
    length = math.sqrt(dx**2 + dy**2 + dz**2)
    dx /= length
    dy /= length
    dz /= length

    # Update coordinates based on forward or backward movement
    if forward:
        x_new = x + distance * dx
        y_new = y + distance * dy
        z_new = z + distance * dz
    else:
        x_new = x - distance * dx
        y_new = y - distance * dy
        z_new = z - distance * dz

    # Return new coordinates
    return [x_new, y_new, z_new, rx, ry, rz]

for i in range(20):
    new_coords = move_robot(coords)
    print(new_coords)
    coords[0] = new_coords[0]
    coords[1] = new_coords[1]
    coords[2] = new_coords[2]
    mc.send_coords(coords, 10)
    time.sleep(1)