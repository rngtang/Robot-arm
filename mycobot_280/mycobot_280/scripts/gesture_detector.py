#!/usr/bin/env python3
# encoding:utf-8

import cv2
import rospy
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from mycobot_communication.msg import HandGesture
from mediapipe.python import Image, ImageFormat
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python.vision.gesture_recognizer import GestureRecognizer, GestureRecognizerOptions


GESTURE_RECOGNIZER_MODEL_PATH = "/home/er/robotic-arm/HandTracking/gesture_recognizer.task"


class GestureDetector:
    DEFAULT_FRAME_RATE = 30

    GESTURES = {
        "None": HandGesture.NONE,
        "Closed_Fist": HandGesture.CLOSED_FIST,
        "Open_Palm": HandGesture.OPEN_PALM,
        "Pointing_Up": HandGesture.POINTING_UP,
        "Thumb_Down": HandGesture.THUMB_DOWN,
        "Thumb_Up": HandGesture.THUMB_UP,
        "Victory": HandGesture.VICTORY,
        "ILoveYou": HandGesture.I_LOVE_YOU
    }

    def __init__(self) -> None:
        rospy.init_node("gesture_detector")

        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=GESTURE_RECOGNIZER_MODEL_PATH),
            min_tracking_confidence=0.1
        )

        self.recognizer = GestureRecognizer.create_from_options(options)

        self.raw_image = None
        self.image = None

        self._cv_bridge = CvBridge()

        self.rate = rospy.Rate(self.DEFAULT_FRAME_RATE)

        rospy.Subscriber("camera/rgb/compressed", CompressedImage, self.update_image)

        self.publisher = rospy.Publisher("cv/hand_gestures", HandGesture, queue_size=10)

    def update_image(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        self.raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        image_rgb = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2RGB)
        self.image = image_rgb

    def detect(self):
        image = Image(image_format=ImageFormat.SRGB, data=self.image)
        results = self.recognizer.recognize(image)

        if results.gestures:
            gesture = results.gestures[0][0].category_name

            if gesture in self.GESTURES:
                msg = HandGesture()
                msg.gesture = self.GESTURES[gesture]
                self.publisher.publish(msg)
            else:
                print(f"Gesture {gesture} not recognized!")

    def run(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                self.detect()
            self.rate.sleep()


if __name__ == '__main__':
    GestureDetector().run()
