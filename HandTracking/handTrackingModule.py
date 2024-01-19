
import cv2
import mediapipe as mp
import time
import random
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord

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
    

def main():
    mc = MyCobot("/dev/ttyAMA0", 1000000)
    mc.send_angles([0, 0, 0, 0, 0, 45], 30)
    cap = cv2.VideoCapture(0)
    tracker = handTracker()
    xCord = 0
    while True:
        success,image = cap.read()
        image = tracker.handsFinder(image)
        lmList = tracker.positionFinder(image)
        # using index 13 for palm data point
        if len(lmList) != 0:
            if lmList[13][1] < 240:
                xCord =+ 5
                mc.send_angles([xCord, 0, 0, 0, 0, 45], 30)
            # if lmList[13][1] > 280:
            #     xCord =- 5
            #     mc.send_angles([xCord, 0, 0, 0, 0, 45], 30)
            # print("------------", lmList, "------------")

        cv2.imshow("Video",image)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
