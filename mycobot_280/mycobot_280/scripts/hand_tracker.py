#!/usr/bin/env python3
# encoding:utf-8

import rospy

from std_msgs.msg import UInt32MultiArray
from pymycobot.mycobot import MyCobot


class HandTracker:
    CENTER_X = 160
    CENTER_Y = 120

    DISTANCE_COEFFICIENT = 0.015
    SPEED_COEFFICIENT = -0.020

    def __init__(self) -> None:
        rospy.init_node("hand_tracker")

        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 30, -30, 0, 0, -135], 40)

        self.j1, self.j2, self.j3, self.j4 = 0, 30, -30, 0
        self.x, self.y = self.CENTER_X, self.CENTER_Y

        rospy.Subscriber("cv/detections", UInt32MultiArray, self.track_hand)

    def track_hand(self, data):
        new_x, new_y = data.data

        if new_x != self.CENTER_X:
            self.j1 += self.DISTANCE_COEFFICIENT * (self.CENTER_X - new_x) + self.SPEED_COEFFICIENT * (new_x - self.x)

        if new_y != self.CENTER_Y:
            self.j4 += self.DISTANCE_COEFFICIENT * (self.CENTER_Y - new_y) + self.SPEED_COEFFICIENT * (new_y - self.y)

        self.mc.send_angles([self.j1, self.j2, self.j3, self.j4, 0, -135], 0)
        self.x, self.y = data.data


if __name__ == '__main__':
    tracker = HandTracker()
    while not rospy.is_shutdown():
        rospy.spin()
