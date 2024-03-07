#!/usr/bin/env python3
# encoding:utf-8

import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class CameraFeedPublisher:
    def __init__(self) -> None:
        rospy.init_node("camera_feed_publisher")

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height

        # Set publisher rate (framerate) to custom framerate
        self.loop_rate = rospy.Rate(self.cap.get(cv2.CAP_PROP_FPS))

        self._cv_bridge = CvBridge()

        self.publisher = rospy.Publisher("camera/rgb/compressed", CompressedImage, queue_size=10)

        self.running = True

    def start(self):
        while not rospy.is_shutdown() and self.running:
            success, image = self.cap.read()
            if not success:
                print("Camera did not receive image feed.")
                continue

            # Convert image read from cv2.videoCapture to Image message to be published
            image_msg = self._cv_bridge.cv2_to_compressed_imgmsg(image)
            self.publisher.publish(image_msg)

            self.loop_rate.sleep()

    def stop(self):
        self.running = False


if __name__ == "__main__":
    camera = CameraFeedPublisher()
    camera.start()
