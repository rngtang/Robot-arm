from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyAMA0", 1000000)

mc.release_all_servos()