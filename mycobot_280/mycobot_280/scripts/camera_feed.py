#!/usr/bin/env python3
# encoding:utf-8

import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class CameraFeedPublisher:
    """
    A class used to launch a ROS (Robot Operating System) node that captures video feed from a camera, 
    compresses it, and publishes it as a ROS topic.

    Attributes:
        cap (cv2.VideoCapture): Video capture object from OpenCV for accessing camera feed.
        loop_rate (rospy.Rate): Rate object from rospy to control the loop rate of the node. Set default to 30 Hz.
        _cv_bridge (CvBridge): CvBridge object for converting between OpenCV images and ROS image messages.
        publisher (rospy.Publisher): ROS publisher object for publishing compressed image messages.

    Methods:
        run(): Function to start capturing frames from the camera, converting them to ROS image messages,
               and publishing them continuously in a loop at the frequency of loop_rate.
    """
    def __init__(self) -> None:
        """Initialize a new instance of the CameraFeedPublisher class."""
        # Initialize ROS node with the name "camera_feed_publisher"
        rospy.init_node("camera_feed_publisher")

        # Initialize video capture object for default camera (channel 0)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # Disable auto exposure
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -200)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, -50)

        # Set publisher rate (framerate) to custom framerate
        self.loop_rate = rospy.Rate(self.cap.get(cv2.CAP_PROP_FPS))

        # Create CvBridge object for image conversion
        self._cv_bridge = CvBridge()

        # Create ROS publisher for the CompressedImage topic "camera/rgb/compressed"
        self.publisher = rospy.Publisher("camera/rgb/compressed", CompressedImage, queue_size=10)

    def run(self):
        """
        Capture frames from the camera, convert them to ROS Image messages,
        and publish them continuously in a loop at the frequency of loop_rate.
        """
        # Loop until ROS shutdown or node is stopped
        while not rospy.is_shutdown():
            # Read frame from camera
            success, image = self.cap.read()
            if not success:
                print("Camera did not receive image feed.")
                continue

            # Convert image read from cv2.videoCapture to Image message to be published
            image_msg = self._cv_bridge.cv2_to_compressed_imgmsg(image)
            self.publisher.publish(image_msg)

            # Sleep to maintain loop rate
            self.loop_rate.sleep()


if __name__ == "__main__":
    CameraFeedPublisher().run()
