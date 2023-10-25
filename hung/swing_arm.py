from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

mc = MyCobot("/dev/ttyAMA0", 1000000)

# By passing the angle parameter, let each joint move to the position corresponding to [0, 0, 0, 0, 0, 0]
mc.send_angles([0, 0, 0, 0, 0, 0], 50)

# Set the waiting time to ensure that the robotic arm has reached the specified position
time.sleep(2.5)

# Move joint 1 to the 90 position
mc.send_angle(Angle.J1.value, 90, 50)
# Set the waiting time to ensure that the robotic arm has reached the specified position
time.sleep(2)

# The following code can make the robotic arm swing left and right
# set the number of loops
for _ in range(5):

    # Move joint 2 to the 50 position
    mc.send_angle(Angle.J4.value, 50, 50)

    # Set the waiting time to ensure that the robotic arm has reached the specified position
    time.sleep(1.5)

    # Move joint 2 to the -50 position
    mc.send_angle(Angle.J4.value, -50, 50)

    # Set the waiting time to ensure that the robotic arm has reached the specified position
    time.sleep(1.5)

mc.send_angles([0, 0, 0, 0, 0, 0], 50)

# Set the waiting time to ensure that the robotic arm has reached the specified position
time.sleep(2.5)

# Let the robotic arm relax, you can manually swing the robotic arm
mc.release_all_servos()
