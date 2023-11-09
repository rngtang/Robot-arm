from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD 
import time, sys

sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
from controls import Controls
controls = Controls()

def check(mc):
    print("Check IO part of api\n")
    # Check if the gripper is moving
    flag = mc.is_gripper_moving()
    print("Is gripper moving?: {}".format(flag))

def grab(mc):
    print("grabbing")

    print("open")
    mc.set_gripper_state(0, 70)
    time.sleep(2)

    print("close")
    mc.set_gripper_state(1, 70)
    time.sleep(2)

if __name__ == "__main__":
    print("hello world")
    mc = MyCobot(PI_PORT, PI_BAUD)
    # make it move to zero position
    # mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    time.sleep(1)
    controls.send_angles([0.519999980927, -126.379997253, 17.3099994659, 103.349998474, 39.5499992371, -44.7299995422], 70)
    time.sleep(1)
    grab(mc)


# Joint 1: 0.519999980927
# Joint 2: -126.379997253
# Joint 3: 17.3099994659
# Joint 4: 103.349998474
# Joint 5: 39.5499992371
# Joint 6: -44.7299995422
