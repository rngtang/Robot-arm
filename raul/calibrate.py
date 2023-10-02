from pymycobot.mycobot import MyCobot

mc = MyCobot("/dev/ttyAMA0", 1000000)

mc.send_angles([-50, 2, 0, 0, -8, 1], 70)

for i in range(1, 7):
    mc.set_servo_calibration(i)