import cv2
import mediapipe as mp
import time
import random
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord

import numpy as np
import threading

# import sys
# sys.path.append('/home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_280/mycobot_280/scripts')
# from pymycobot.mycobot import MyCobot
# from pymycobot.genre import Angle
# from pymycobot import PI_PORT, PI_BAUD
# from pymycobot.mypalletizer import MyPalletizer
# from pymycobot.genre import Coord
# from controls import Controls
# controls = Controls()
# controls.test_controls()

# source https://www.section.io/engineering-education/creating-a-hand-tracking-module/

class handTracker():
    """
    A class used to detect and track hands in an image using the Mediapipe library.

    Attributes:
    mode (bool): Whether to detect hands in static images or in real-time video streams.
    maxHands (int): Maximum number of hands to detect.
    detectionCon (float): Minimum confidence value ([0.0, 1.0]) for hand detection to be considered successful.
    modelComplexity (int): Complexity of the detection model ([0, 2]).
    trackCon (float): Minimum confidence value ([0.0, 1.0]) for hand tracking to be considered successful.
    mpHands (mediapipe.solutions.hands): The Mediapipe Hands solution object.
    hands (mediapipe.solutions.hands.Hands): The Mediapipe Hands object used for detection and tracking.
    mpDraw (mediapipe.solutions.drawing_utils): The Mediapipe drawing utilities object.

    Methods:
    handsFinder(image, draw=True): Detects hands in the given image and returns the image with landmarks drawn on it.
    positionFinder(image, handNo=0, draw=True): Returns a list of landmark positions for the specified hand in the given image.
    """

    def __init__(self, mode=False, maxHands=1, detectionCon=0.5, modelComplexity=1, trackCon=0.5):
        """
        Initializes a new instance of the handTracker class.

        Args:
        mode (bool): Whether to detect hands in static images or in real-time video streams.
        maxHands (int): Maximum number of hands to detect.
        detectionCon (float): Minimum confidence value ([0.0, 1.0]) for hand detection to be considered successful.
        modelComplexity (int): Complexity of the detection model ([0, 2]).
        trackCon (float): Minimum confidence value ([0.0, 1.0]) for hand tracking to be considered successful.
        """
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def handsFinder(self,image,draw=True):
        """
        Detects hands in the given image and returns the image with landmarks drawn on it.

        Args:
        image (numpy.ndarray): The image to detect hands in.
        draw (bool): Whether to draw landmarks on the image.

        Returns:
        numpy.ndarray: The image with landmarks drawn on it.
        """
        imageRGB = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:

                if draw:
                    self.mpDraw.draw_landmarks(image, handLms, self.mpHands.HAND_CONNECTIONS)
        return image
    
    def positionFinder(self, image, handNo=0):
        """
        Returns a list of landmark positions for the specified hand in the given image.

        Args:
            image (numpy.ndarray): The image to detect hand landmarks in.
            handNo (int): The index of the hand to detect landmarks for.
            draw (bool): Whether to draw a circle around the detected landmarks.

        Returns:
            list: A list of landmark positions for the specified hand.
        """
        lmList = []
        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[handNo]
            for id, landmark in enumerate(hand.landmark):
                h, w, c = image.shape
                cx, cy = int(landmark.x * w), int(landmark.y * h)
                cz = landmark.z # add z-coordinate
                lmList.append([id, cx, cy, cz]) # append x, y, and z coordinates to list
        return lmList

class CameraFlangeController:
    def __init__(self):
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 0, 0, 0, 0, -135], 40)
        self.cap = cv2.VideoCapture(0)
        #lower res means faster tracking
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height
        self.tracker = handTracker()
        self.j1, self.j4 = 0, 0
        self.running = True

    def start(self):
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop(self):
        self.running = False

    def control_loop(self):
        while self.running:
            success, image = self.cap.read()
            if not success:
                continue
            
            image = self.tracker.handsFinder(image)
            lmList = self.tracker.positionFinder(image)
            
            if len(lmList) >= 12:
                x, y, z = lmList[13][1], lmList[13][2], lmList[13][3]
                if x < 155 or x > 155:
                    self.j1 -= 0.01 * (x - 155)
                if y < 107 or y > 107:
                    self.j4 -= 0.01 * (y - 107)
                # Send joint angles to MyCobot
                self.mc.send_angles([self.j1, 0, 0, self.j4, 0, -135], 80)
            
            cv2.imshow("Video", image)
            cv2.waitKey(1)
    
def main():
    mc = MyCobot("/dev/ttyAMA0", 1000000)
    mc.send_angles([0, 0, 0, 0, 0, -135], 40)
    cap = cv2.VideoCapture(0)
    tracker = handTracker()
    j1, j2, j3, j4 = 0, 0, 0, 0
    while True:
        success,image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        #using index 13 for point 13 near center of hand. This index's location on hand may vary if point 13 is not at index 13 in lmList.       
        if len(lmList) != 0:
            x, y, z = lmList[13][1], lmList[13][2], lmList[13][3]
            if  x < 290 or x > 310:
                j1 = j1 - 0.007*(x - 300)
            #if z < -0.09 and j2 < 90:
                #j2 = j2 - 40*(z + 0.055)
                #j3 = j3 + 40*(z + 0.055)
            #if z > -0.02 and j2 > -90:
                #j2 = j2 - 40*(z + 0.055)
                #j3 = j3 + 40*(z + 0.055)
            if y < 205 or y > 225:
                j4 = j4 - 0.007*(y - 215)
            mc.send_angles([j1, j2, j3, j4, 0, -135], 100)
            #print("------------", lmList, "------------")
        cv2.imshow("Video",image)
        cv2.waitKey(1)

if __name__ == "__main__":
    #tracking without concurrency and additional imporvements
    #main()

    #tracking with concurrency and additional improvements
    controller = CameraFlangeController()
    controller.start()
    input("Press Enter to stop...")
    controller.stop()
