#!/usr/bin/env python3
# encoding:utf-8

import rospy

from enum import IntEnum
from std_msgs.msg import UInt32MultiArray
from pymycobot.mycobot import MyCobot
from mycobot_communication.msg import HandGesture


class Gesture(IntEnum):
    """The 8 hand gestures."""
    NONE = 0
    CLOSED_FIST = 1
    OPEN_PALM = 2
    POINTING_UP = 3
    THUMB_DOWN = 4
    THUMB_UP = 5
    VICTORY = 6
    I_LOVE_YOU = 7


class HandTracker:
    """
    A class used to track hand positions and hand gestures and control the MyCobot arm accordingly.

    Attributes:
        CENTER_X (int): X-coordinate of the center position of the frame for tracking.
        CENTER_Y (int): Y-coordinate of the center position of the frame for tracking.
        DISTANCE_COEFFICIENT (float): Coefficient for adjusting the MyCobot arm position based on hand distance.
        SPEED_COEFFICIENT (float): Coefficient for adjusting the MyCobot arm position based on the speed of movement.
        MAX_GESTURE_MULTIPLIER (int): Maximum multiplier for adjusting hand gesture-based movement.
        GESTURE_COEFFICIENT (int): Coefficient for adjusting the movement based on hand gestures.
        DEFAULT_TRACKER_RATE (int): Default rate for the tracker loop.
        mc (pymycobot.mycobot.MyCobot): MyCobot object for controlling the robotic arm.
        j1, j2, j3, j4 (int): Joint angles of the robotic arm.
        x, y (int): Current position of the tracked hand.
        prev_gesture (Gesture): Previous detected hand gesture.
        new_x, new_y (int): New position of the tracked hand.
        gesture (Gesture): New detected hand gesture.
        rate (rospy.Rate): Rate object from rospy to control the loop rate of the node. Set default to 30 Hz.

    Methods:
        update_hand(self, data): Callback function to receive and update hand position data.
        update_gesture(self, data): Callback function to receive and update hand position data.
        track(self): Function to track hand position and gesture and update the MyCobot arm positions accordingly.
        run(self): Run hand tracking and MyCobot arm control continuously in a loop at the frequency of loop_rate.
    """
    CENTER_X = 160
    CENTER_Y = 120

    DISTANCE_COEFFICIENT = 0.010
    SPEED_COEFFICIENT = -0.010

    MAX_GESTURE_MULTIPLIER = 5
    GESTURE_COEFFICIENT = 2

    DEFAULT_TRACKER_RATE = 30

    def __init__(self) -> None:
        """Initialize a new instance of the HandTracker class."""
        # Initialize ROS node with the name "hand_tracker"
        rospy.init_node("hand_tracker")

        # Create MyCobot object
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        # Send the initial joint angles
        self.mc.send_angles([0, 30, -30, 0, 0, -135], 40)

        # Initialize joint angles, hand position, and hand gesture variables
        self.j1, self.j2, self.j3, self.j4 = 0, 30, -30, 0
        self.x, self.y = self.CENTER_X, self.CENTER_Y
        self.prev_gesture = Gesture.NONE

        self.new_x = None
        self.new_y = None
        self.gesture = None

        # Set loop rate to DEFAULT_TRACKER_RATE
        self.rate = rospy.Rate(self.DEFAULT_TRACKER_RATE)

        # ROS Subscriber for receiving ROS hand coordinate messages from the topic "cv/detections" which calls
        # self.update_hand with the message as the callback function
        rospy.Subscriber("cv/detections", UInt32MultiArray, self.update_hand)

        # ROS Subscriber for receiving ROS HandGesture messages from the topic "cv/hand_gestures" which calls
        # self.update_gesture with the message as the callback function
        rospy.Subscriber("cv/hand_gestures", HandGesture, self.update_gesture)

    def update_hand(self, data):
        """
        Callback function to receive and update hand position data.

        Parameters:
            data (std_msgs.msg.UInt32MultiArray): ROS hand position message received from the ROS topic.
        """
        self.new_x, self.new_y = data.data

    def update_gesture(self, data):
        """
        Callback function to receive and update hand gesture data.

        Parameters:
            data (mycobot_communication.msg.HandGesture): ROS HandGesture message received from the ROS topic.
        """
        self.gesture = Gesture(data.gesture)

    def track(self):
        """Function to track hand position and gesture and control the MyCobot arm accordingly."""
        # Adjust MyCobot arm joints based on detected hand landmarks
        new_x, new_y = self.new_x, self.new_y

        if new_x and new_y:
            # Calculate adjustments to the j1 and j4 joints based on
            # hand distance from the center and speed of hand movement
            self.j1 += self.DISTANCE_COEFFICIENT * (self.CENTER_X - new_x) + self.SPEED_COEFFICIENT * (new_x - self.x)
            self.j4 += self.DISTANCE_COEFFICIENT * (self.CENTER_Y - new_y) + self.SPEED_COEFFICIENT * (new_y - self.y)

            # Update the current hand position
            self.x, self.y = new_x, new_y

        # Adjust MyCobot arm joints based on detected hand gesture
        gesture = self.gesture

        if (gesture == Gesture.THUMB_UP and self.j2 > -90) or (gesture == Gesture.POINTING_UP and self.j2 < 90):
            # If the same gesture has been detected for a few frames, the MyCobot arm moves faster
            if self.prev_gesture == gesture:
                self.multiplier += 1
                self.multiplier = min(self.multiplier, self.MAX_GESTURE_MULTIPLIER)
            else:
                self.multiplier = 1

            direction = 1 if gesture == Gesture.THUMB_UP else -1

            # Calculate adjustments to the j2 and j3 joints based on the detected hand gesture and current multiplier
            self.j2 -= direction * self.multiplier * self.GESTURE_COEFFICIENT
            self.j3 += direction * self.multiplier * self.GESTURE_COEFFICIENT

        else:
            # No gesture of interest detected, reset self.multiplier to 1
            self.multiplier = 1

        # Update the previous detected gesture
        self.prev_gesture = gesture

    def run(self):
        """Run hand tracking and MyCobot arm control continuously."""
        # Loop until ROS shutdown or node is stopped
        while not rospy.is_shutdown():
            # Perform hand tracking and update the MyCobot arm joints
            self.track()

            # Send adjusted joint angles to the MyCobot arm for arm movement
            self.mc.send_angles([self.j1, self.j2, self.j3, self.j4, 0, -135], 0)

            # Sleep to maintain loop rate
            self.rate.sleep()


if __name__ == '__main__':
    HandTracker().run()
