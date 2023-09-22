from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of mycobot, these two variables can be referenced to initialize MyCobot
import time

# Initialize a MyCobot object
mc = MyCobot(PI_PORT, PI_BAUD)
# Get the coordinates of the current location
angle_datas = mc.get_angles()
print(angle_datas)


#By passing the angle parameter, let each joint of the robotic arm move to the position
mc.send_angles([0, 0, 0, 0, 0, 0], 50)
print(mc.is_paused())
# Set the waiting time to ensure that the robotic arm has reached the specified position
# while not mc.is_paused():
time.sleep(2.5)

# Move joint 1 to the 90 position
mc.send_angle(Angle.J1.value, 90, 50)

# Set the waiting time to ensure that the robotic arm has reached the specified position
time.sleep(2)

# set loop times
num = 5

# The following code can make the robotic arm swing left and right
while num > 0:
    # Move joint 2 to the 50 position
    mc.send_angle(Angle.J2.value, 50, 50)

    # Set the waiting time to ensure that the robotic arm has reached the specified position
    time.sleep(1.5)

    # Move joint 2 to the -50 position
    mc.send_angle(Angle.J2.value, -50, 50)

    # Set the waiting time to ensure that the robotic arm has reached the specified position
    time.sleep(1.5)

    num -= 1

#  Make the robotic arm retract. You can manually swing the robotic arm, and then use the get_angles() function to get the coordinate sequence, use this function to let the robotic arm reach the position you want.
mc.send_angles([88.68, -138.51, 155.65, -128.05, -9.93, -15.29], 50)

# Set the waiting time to ensure that the robotic arm has reached the specified position
time.sleep(2.5)

# Let the robotic arm relax, you can manually swing the robotic arm
mc.release_all_servos()
