#!/usr/bin/env python3
# encoding:utf-8

import rospy

from enum import IntEnum
from std_msgs.msg import UInt32MultiArray
from pymycobot.mycobot import MyCobot
from mycobot_communication.msg import HandGesture


class Gesture(IntEnum):
    NONE = 0
    CLOSED_FIST = 1
    OPEN_PALM = 2
    POINTING_UP = 3
    THUMB_DOWN = 4
    THUMB_UP = 5
    VICTORY = 6
    I_LOVE_YOU = 7


class HandTracker:
    CENTER_X = 160
    CENTER_Y = 120

    DISTANCE_COEFFICIENT = 0.010
    SPEED_COEFFICIENT = -0.010

    MAX_GESTURE_MULTIPLIER = 5
    GESTURE_COEFFICIENT = 2

    DEFAULT_TRACKER_RATE = 30

    def __init__(self) -> None:
        rospy.init_node("hand_tracker")

        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 30, -30, 0, 0, -135], 40)

        self.j1, self.j2, self.j3, self.j4 = 0, 30, -30, 0
        self.x, self.y = self.CENTER_X, self.CENTER_Y
        self.prev_gesture = Gesture.NONE

        self.new_x = None
        self.new_y = None
        self.gesture = None

        self.rate = rospy.Rate(self.DEFAULT_TRACKER_RATE)

        rospy.Subscriber("cv/detections", UInt32MultiArray, self.update_hand)
        rospy.Subscriber("cv/hand_gestures", HandGesture, self.update_gesture)

    def update_hand(self, data):
        self.new_x, self.new_y = data.data

    def update_gesture(self, data):
        self.gesture = Gesture(data.gesture)

    def track(self):
        new_x, new_y = self.new_x, self.new_y

        if new_x and new_y:
            self.j1 += self.DISTANCE_COEFFICIENT * (self.CENTER_X - new_x) + self.SPEED_COEFFICIENT * (new_x - self.x)
            self.j4 += self.DISTANCE_COEFFICIENT * (self.CENTER_Y - new_y) + self.SPEED_COEFFICIENT * (new_y - self.y)

            self.x, self.y = new_x, new_y

        gesture = self.gesture

        if (gesture == Gesture.THUMB_UP and self.j2 > -90) or (gesture == Gesture.POINTING_UP and self.j2 < 90):
            if self.prev_gesture == gesture:
                self.multiplier += 1
                self.multiplier = min(self.multiplier, self.MAX_GESTURE_MULTIPLIER)
            else:
                self.multiplier = 1

            direction = 1 if gesture == Gesture.THUMB_UP else -1

            self.j2 -= direction * self.multiplier * self.GESTURE_COEFFICIENT
            self.j3 += direction * self.multiplier * self.GESTURE_COEFFICIENT

        else:
            self.multiplier = 1

        self.prev_gesture = gesture

    def run(self):
        while not rospy.is_shutdown():
            self.track()
            self.mc.send_angles([self.j1, self.j2, self.j3, self.j4, 0, -135], 0)
            self.rate.sleep()


if __name__ == '__main__':
    HandTracker().run()
