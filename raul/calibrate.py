from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyAMA0", 1000000)

# mc.send_angles([-50, 2, 0, 0, -8, 1], 70)
mc.send_angles([0,0,0,0,0,0], 70)

# for i in range(1, 7):
#     mc.set_servo_calibration(i)

time.sleep(3)
print(mc.get_angles())
time.sleep(3)

# mc.release_all_servos()