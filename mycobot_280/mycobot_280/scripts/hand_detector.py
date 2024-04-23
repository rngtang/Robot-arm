#!/usr/bin/env python3
# encoding:utf-8

import cv2
import rospy
import numpy as np

from enum import IntEnum
from mediapipe.python.solutions.hands import Hands
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt32MultiArray


class HandLandmark(IntEnum):
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
        hands (mediapipe.solutions.hands.Hands): The Mediapipe Hands object used for detection and tracking.
        raw_image: Current raw image received from the camera.
        image: Current converted RGB image used for processing.
        visualizer_publisher (rospy.Publisher): ROS Publisher for publishing visualized detections.
        detections_publisher (rospy.Publisher): ROS Publisher for publishing coordinates of raw detections.
        loop_rate (rospy.Rate): Rate object from rospy to control the loop rate of the node. Set default to 30 Hz.
        _cv_bridge (CvBridge): CvBridge object for converting between OpenCV images and ROS image messages.

    Methods:
        update_image(image_msg): Callback function to receive and convert ROS Image messages to Cv2 images.
        detect(hand_index=0): Function to detect hands in the given image and publish visualized detections.
        run(): Main function of the class to run hand detection continuously in a loop at the frequency of loop_rate.
    """
    # Default frame rate
    DEFAULT_FRAME_RATE = 30

    def __init__(self, static_image_mode=False, max_num_hands=1, detection_confidence=0.5,
                 model_complexity=0, tracking_confidence=0.1) -> None:
        """
        Initialize a new instance of the HandTracker class.

        Parameters:
            static_image_mode (bool): Whether to detect hands in static images or in real-time video streams.
            max_num_hands (int): Maximum number of hands to detect.
            detection_confidence (float): Minimum confidence value ([0.0, 1.0]) for
                                          hand detection to be considered successful.
            model_complexity (int): Complexity of the detection model ([0, 2]).
            track_confidence (float): Minimum confidence value ([0.0, 1.0])
                                      for hand tracking to be considered successful.
        """
        # Initialize ROS node with the name "hand_detector"
        rospy.init_node("hand_detector")

        # Initialize Mediapipe Hands object
        self.hands = Hands(static_image_mode, max_num_hands, model_complexity,
                           detection_confidence, tracking_confidence)

        self.raw_image = None  # Raw image from camera
        self.image = None  # Converted image (RGB) from camera

        # Create CvBridge object for image conversion
        self._cv_bridge = CvBridge()

        # Set loop rate (framerate) to DEFAULT_FRAME_RATE
        self.rate = rospy.Rate(self.DEFAULT_FRAME_RATE)

        # ROS Subscriber for receiving CompressedImage messages from the topic "camera/rgb/compressed"
        rospy.Subscriber("camera/rgb/compressed", CompressedImage, self.update_image)

        # ROS Publisher for publishing visualized detections (images with overlaid bounding boxes)
        # to the Image topic "cv/detections_visualized"
        self.visualizer_publisher = rospy.Publisher("cv/detections_visualized", Image, queue_size=10)

        # ROS Publisher for publishing coordinates of raw detections to the topic "cv/detections"
        self.detections_publisher = rospy.Publisher("cv/detections", UInt32MultiArray, queue_size=10)

    def update_image(self, image_msg):
        """Callback function to receive and convert ROS CompressedImage messages to Cv2 images."""
        # Convert the CompressedImage message to Cv2 Image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        self.raw_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert the image to RGB for detection. Both the self.raw_image and self.image needs to be saved since we need
        # the self.raw_image (with the original BGR color scheme) for publishing visualized detections, while the
        # Mediapipe model uses the RGB color scheme
        image_rgb = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2RGB)
        self.image = image_rgb

    def detect(self, hand_index=0):
        """
        Detect hand landmarks in the given image and publish visualized detections.

        Parameters:
            hand_index (int): The index of the hand to detect landmarks for.
        """
        image = self.image
        raw_image = self.raw_image

        # Process image using Mediapipe Hands
        height, width, _ = image.shape
        results = self.hands.process(image)

        # If hand landmarks are detected
        if results.multi_hand_landmarks:
            x_min, y_min, x_max, y_max = float('inf'), float('inf'), 0, 0
            hand = results.multi_hand_landmarks[hand_index].landmark

            # Extract the position of the palm center as the midpoint of the WRIST and MIDDLE_FINGER_MCP
            if len(hand) > max(HandLandmark.WRIST.value, HandLandmark.MIDDLE_FINGER_MCP.value):
                wrist = hand[HandLandmark.WRIST.value]
                middle = hand[HandLandmark.MIDDLE_FINGER_MCP.value]

                palm_x = int((wrist.x + middle.x) * width / 2)
                palm_y = int((wrist.y + middle.y) * height / 2)

                if palm_x >= 0 and palm_y >= 0:
                    # Create and publish the ROS Message for palm coordinates
                    msg = UInt32MultiArray()
                    msg.data = [palm_x, palm_y]
                    self.detections_publisher.publish(msg)

            # Draw bounding box around hand
            for landmark in hand:
                global_x, global_y = int(landmark.x * width), int(landmark.y * height)

                x_min = min(x_min, global_x)
                x_max = max(x_max, global_x)
                y_min = min(y_min, global_y)
                y_max = max(y_max, global_y)

            cv2.rectangle(raw_image, (x_min, y_min), (x_max, y_max), (0, 102, 0), 2)

        # Create and publish the ROS Image message for the image with overlaid bounding boxes
        image_msg = self._cv_bridge.cv2_to_imgmsg(raw_image)
        self.visualizer_publisher.publish(image_msg)

    def run(self):
        """Run hand detection continuously."""
        # Loop until ROS shutdown or node is stopped
        while not rospy.is_shutdown():
            if self.image is not None:
                # Perform hand detection
                self.detect()

            # Sleep to maintain loop rate
            self.rate.sleep()


if __name__ == '__main__':
    HandDetector().run()
