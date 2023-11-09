# code from Elephant Robotics: https://docs.elephantrobotics.com/docs/gitbook-en/7-ApplicationBasePython/7.7_example.html

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of mycobot, these two variables can be referenced to initialize MyCobot
import time


def gripper_test(mc):
    print("Start check IO part of api\n")
    # Check if the gripper is moving
    flag = mc.is_gripper_moving()
    print("Is gripper moving: {}".format(flag))
    time.sleep(1)

    # Set the current position to (2048).
    # Use it when you are sure you need it.
    # Gripper has been initialized for a long time. Generally, there
    # is no need to change the method.
    # mc.set_gripper_ini()
    # Set joint point 1 to rotate to the position of 2048
    mc.set_encoder(1, 2048)
    time.sleep(2)

    # Set six joint positions and let the robotic arm rotate to this position at a speed of 20
    mc.set_encoders([1024, 1024, 1024, 1024, 1024, 1024], 20)
    time.sleep(3)

    #Get the position information of joint point 1
    print(mc.get_encoder(1))
    # Set the gripper to rotate to the position of 2048
    mc.set_encoder(7, 2048)
    time.sleep(3)
    # Set the gripper to rotate to the position of 1300
    mc.set_encoder(7, 1300)
    time.sleep(3)

    # Let the gripper reach the state of 2048 at a speed of 70, 2048 will report an error, so change it to 255
    mc.set_gripper_value(255, 70)
    time.sleep(3)
    # Let the gripper reach the state of 1500 at a speed of 70, 1500 will report an error, so change it to 255
    mc.set_gripper_value(255, 70)
    time.sleep(3)


    num=5
    while num>0:
        # Set the state of the gripper to quickly open the gripper at a speed of 70
        mc.set_gripper_state(0, 70)
        time.sleep(3)
        # Set the state of the gripper to quickly close the gripper at a speed of 70
        mc.set_gripper_state(1, 70)
        time.sleep(3)
        num-=1

    # Get the value of the gripper
    print("")
    print(mc.get_gripper_value())

if __name__ == "__main__":
    # MyCobot class initialization requires two parameters:
    #   The first is the serial port string, such as:
    #       linux: "/dev/ttyUSB0"
    #          or "/dev/ttyAMA0"
    #       windows: "COM3"
    #   The second is the baud rate::
    #       M5 version is: 115200
    #
    #    such as:
    #       mycobot-M5:
    #           linux:
    #              mc = MyCobot("/dev/ttyUSB0", 115200)
    #          or mc = MyCobot("/dev/ttyAMA0", 115200)
    #           windows:
    #              mc = MyCobot("COM3", 115200)
    #       mycobot-raspi:
    #           mc = MyCobot(PI_PORT, PI_BAUD)
    #
    # Initialize a MyCobot object
    # Create object code for Raspberry Pi version below
    mc = MyCobot(PI_PORT, PI_BAUD)
    # make it move to zero position
    mc.set_encoders([2048, 2048, 2048, 2048, 2048, 2048], 20)
    time.sleep(3)
    gripper_test(mc)
