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

    # ready position
    controls.send_angles([4.21000003815, 1.23000001907, -90.6100006104, -3.16000008583, 96.4100036621, -149.940002441], 60)
    time.sleep(1)


    # go to grab
    print("open")
    mc.set_gripper_state(0, 70)
    time.sleep(2)
    controls.send_angles([4.21000003815, -58.9700012207, -78.9199981689, 31.9899997711, 86.6600036621, -152.660003662], 60)
    time.sleep(2)

    # grab
    print("close")
    mc.set_gripper_state(1, 70)
    time.sleep(2)

    # finished
    controls.send_angles([4.13000011444, 32.25, -79.3600006104, 48.8600006104, -0.610000014305, -40.7799987793], 60)
    time.sleep(1)

if __name__ == "__main__":
    print("hello world")
    mc = MyCobot(PI_PORT, PI_BAUD)
    # make it move to zero position
    # mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
    controls.send_angles([0, 0, 0, 0, 0, 0], 70)
    time.sleep(2)
    
    grab(mc)



