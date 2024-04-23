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
    """
    A class used to detect hand gestures in an image using the Mediapipe library.

    Attributes:
        DEFAULT_FRAME_RATE (int): Default frame rate for processing.
        GESTURES (dict): Dictionary mapping Mediapipe gesture names to corresponding HandGesture ROS message constants.
        recognizer (mediapipe.tasks.vision.gesture_recognizer.GestureRecognizer): The Mediapipe GestureRecognizer object
                                                                                  used for gesture detection.
        image(numpy.ndarray): Current converted RGB image used for processing.
        _cv_bridge (CvBridge): CvBridge object for converting between OpenCV images and ROS image messages.
        loop_rate (rospy.Rate): Rate object from rospy to control the loop rate of the node. Set default to 30 Hz.
        publisher (rospy.Publisher): ROS Publisher for publishing detected gestures.

    Methods:
        update_image(image_msg): Callback function to receive and convert ROS Image messages to Cv2 images.
        detect(): Function to detect gestures in the given image and publish the detected gesture.
        run(): Main function of the class to run hand detection continuously in a loop at the frequency of loop_rate.
    """
    # Default frame rate
    DEFAULT_FRAME_RATE = 30

    # Dictionary mapping Mediapipe gesture names to corresponding HandGesture ROS message constants
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
        """Initialize a new instance of the GestureDetector class."""
        # Initialize ROS node with the name "gesture_detector"
        rospy.init_node("gesture_detector")

        # Configure GestureRecognizerOptions
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=GESTURE_RECOGNIZER_MODEL_PATH),
            min_tracking_confidence=0.1
        )

        # Create Mediapipe GestureRecognizer object
        self.recognizer = GestureRecognizer.create_from_options(options)

        self.image = None  # Converted image (RGB) from camera

        # Create CvBridge object for image conversion
        self._cv_bridge = CvBridge()

        # Set loop rate (framerate) to DEFAULT_FRAME_RATE
        self.rate = rospy.Rate(self.DEFAULT_FRAME_RATE)

        # ROS Subscriber for receiving CompressedImage messages from the topic "camera/rgb/compressed" which calls
        # self.update_image with the message as the callback function
        rospy.Subscriber("camera/rgb/compressed", CompressedImage, self.update_image)

        # ROS Publisher for publishing detected gestures to the HandGesture topic "cv/hand_gestures"
        self.publisher = rospy.Publisher("cv/hand_gestures", HandGesture, queue_size=10)

    def update_image(self, image_msg):
        """
        Callback function to receive and convert ROS CompressedImage messages to Cv2 images.

        Parameters:
            image_msg (sensor_msgs.msg.CompressedImage): ROS CompressedImage message received from the ROS topic.
        """
        # Convert the CompressedImage message to a Cv2 Image and convert the image to RGB for detection
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        self.image = cv2.cvtColor(cv2.imdecode(np_arr, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

    def detect(self):
        """Detect hand gestures in the given image and publish the detected gesture."""
        # Create the Mediapipe Image object for detection
        image = Image(image_format=ImageFormat.SRGB, data=self.image)

        # Process hand gestures using Mediapipe GestureRecognizer
        results = self.recognizer.recognize(image)

        # If hand gestures are detected
        if results.gestures:
            # Get detected gesture name
            gesture = results.gestures[0][0].category_name

            if gesture in self.GESTURES:
                # Create and publish the ROS HandGesture Message for the detected gesture
                msg = HandGesture()
                msg.gesture = self.GESTURES[gesture]
                self.publisher.publish(msg)
            else:
                print(f"Gesture {gesture} not recognized!")

    def run(self):
        """Run gesture detection continuously."""
        # Loop until ROS shutdown or node is stopped
        while not rospy.is_shutdown():
            if self.image is not None:
                # Perform gesture detection
                self.detect()

            # Sleep to maintain loop rate
            self.rate.sleep()


if __name__ == '__main__':
    GestureDetector().run()
