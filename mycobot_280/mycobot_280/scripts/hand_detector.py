#!/usr/bin/env python3
# encoding:utf-8

import cv2
import enum
import rospy
import numpy as np

from mediapipe.python.solutions.hands import Hands
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt32MultiArray


class HandLandmark(enum.IntEnum):
    """The 21 hand landmarks."""
    WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_IP = 3
    THUMB_TIP = 4
    INDEX_FINGER_MCP = 5
    INDEX_FINGER_PIP = 6
    INDEX_FINGER_DIP = 7
    INDEX_FINGER_TIP = 8
    MIDDLE_FINGER_MCP = 9
    MIDDLE_FINGER_PIP = 10
    MIDDLE_FINGER_DIP = 11
    MIDDLE_FINGER_TIP = 12
    RING_FINGER_MCP = 13
    RING_FINGER_PIP = 14
    RING_FINGER_DIP = 15
    RING_FINGER_TIP = 16
    PINKY_MCP = 17
    PINKY_PIP = 18
    PINKY_DIP = 19
    PINKY_TIP = 20


class HandDetector:
    """
    A class used to detect and track hands in an image using the Mediapipe library.

    Attributes:
        static_image_mode (bool): Whether to detect hands in static images or in real-time video streams.
        max_num_hands (int): Maximum number of hands to detect.
        detection_confidence (float): Minimum confidence value ([0.0, 1.0]) for hand detection to be successful.
        model_complexity (int): Complexity of the detection model ([0, 2]).
        tracking_confidence (float): Minimum confidence value ([0.0, 1.0]) for hand tracking to be successful.
        mp_hands (mediapipe.solutions.hands): The Mediapipe Hands solution object.
        hands (mediapipe.solutions.hands.Hands): The Mediapipe Hands object used for detection and tracking.
        mp_draw (mediapipe.solutions.drawing_utils): The Mediapipe drawing utilities object.

    Methods:
        find_hands(image, draw=True): Detects hands in the given image and returns the image with landmarks drawn on it.
        find_positions(image, hand_number=0, draw=True): Returns a list of landmark positions 
                                                         for the specified hand in the given image.
    """
    DEFAULT_FRAME_RATE = 30

    def __init__(self, static_image_mode=False, max_num_hands=1, detection_confidence=0.5,
                 model_complexity=0, tracking_confidence=0.1) -> None:
        """
        Initializes a new instance of the HandTracker class.

        Parameters:
            static_image_mode (bool): Whether to detect hands in static images or in real-time video streams.
            max_num_hands (int): Maximum number of hands to detect.
            detectionCon (float): Minimum confidence value ([0.0, 1.0]) for hand detection to be considered successful.
            modelComplexity (int): Complexity of the detection model ([0, 2]).
            trackCon (float): Minimum confidence value ([0.0, 1.0]) for hand tracking to be considered successful.
        """
        rospy.init_node("hand_detector")

        self.hands = Hands(static_image_mode, max_num_hands, model_complexity,
                           detection_confidence, tracking_confidence)

        self.raw_image = None
        self.image = None

        self._cv_bridge = CvBridge()

        self.rate = rospy.Rate(self.DEFAULT_FRAME_RATE)

        rospy.Subscriber("camera/rgb/compressed", CompressedImage, self.update_image)

        self.visualizer_publisher = rospy.Publisher("cv/detections_visualized", Image, queue_size=10)
        self.detections_publisher = rospy.Publisher("cv/detections", UInt32MultiArray, queue_size=10)

    def update_image(self, image_msg):
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        self.raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        image_rgb = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2RGB)
        self.image = image_rgb

    def detect(self, hand_index=0):
        """
        Detects hands in the given image and returns the image with landmarks drawn on it.

        Args:
            image (numpy.ndarray): The image to detect hands in.
            hand_index (int): The index of the hand to detect landmarks for.

        Returns:
            numpy.ndarray: The image with landmarks drawn on it.
        """
        image = self.image
        raw_image = self.raw_image

        height, width, _ = image.shape
        results = self.hands.process(image)

        if results.multi_hand_landmarks:
            x_min, y_min, x_max, y_max = float('inf'), float('inf'), 0, 0
            hand = results.multi_hand_landmarks[hand_index].landmark

            if len(hand) > max(HandLandmark.WRIST.value, HandLandmark.MIDDLE_FINGER_MCP.value):
                wrist = hand[HandLandmark.WRIST.value]
                middle = hand[HandLandmark.MIDDLE_FINGER_MCP.value]

                palm_x = int((wrist.x + middle.x) * width / 2)
                palm_y = int((wrist.y + middle.y) * height / 2)

                msg = UInt32MultiArray()
                msg.data = [palm_x, palm_y]
                self.detections_publisher.publish(msg)

            for landmark in hand:
                global_x, global_y = int(landmark.x * width), int(landmark.y * height)

                x_min = min(x_min, global_x)
                x_max = max(x_max, global_x)
                y_min = min(y_min, global_y)
                y_max = max(y_max, global_y)

            cv2.rectangle(raw_image, (x_min, y_min), (x_max, y_max), (0, 102, 0), 2)

        image_msg = self._cv_bridge.cv2_to_imgmsg(raw_image)
        self.visualizer_publisher.publish(image_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                self.detect()
            self.rate.sleep()


if __name__ == '__main__':
    detector = HandDetector().run()
