from pymycobot.mycobot import MyCobot

mc = MyCobot("/dev/ttyAMA0", 1000000)

mc.send_angles([-47, 0, 0, 0, -8, 0], 70)