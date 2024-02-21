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

    def __init__(self, mode=False, maxHands=1, detectionCon=0.5, modelComplexity=0, trackCon=0.1):
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
                lmList.append([id, cx, cy]) #we exclude z since we are not using it for tracking algo
                if len(lmList) >= 10: #since we only use index 9, we do not need the rest of the points
                    break
        return lmList

    def draw_bounding_box(self, image):
        """
        Draws a bounding box around detected hands in the given image.

        Args:
            image (numpy.ndarray): The image to draw the bounding box on.

        Returns:
            tuple: A tuple containing the modified image with bounding boxes drawn and the centers of the bounding boxes.
        """
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imageRGB)

        centers = []
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                x_min, y_min, x_max, y_max = float('inf'), float('inf'), 0, 0
                for landmark in handLms.landmark:
                    h, w, _ = image.shape
                    x, y = int(landmark.x * w), int(landmark.y * h)
                    x_min = min(x_min, x)
                    y_min = min(y_min, y)
                    x_max = max(x_max, x)
                    y_max = max(y_max, y)
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                centers.append((center_x, center_y))
                cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 255), 2)
        return image, centers


class CameraFlangeController:
    def __init__(self):
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 20, -20, 0, 0, -135], 40)
        self.cap = cv2.VideoCapture(0)
        #lower res means faster tracking
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height
        #self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # Disable auto exposure
        #self.cap.set(cv2.CAP_PROP_EXPOSURE, 0)  # Set exposure value (adjust as needed)
        #self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)
        self.tracker = handTracker()
        self.j1, self.j2, self.j3, self.j4 = 0, 20, -20, 0
        self.last_x, self.last_y = 160, 120
        self.success = False
        self.running = True
        self.image = None
        self.success_event = threading.Event()


    def start(self):
        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop(self):
        self.running = False
    
    def camera_loop(self):
        while self.running:
            success, image = self.cap.read()
            if not success:
                continue
            self.success_event.set()
            self.image = image
            self.success = success

    def control_loop(self):
        while self.running:
            self.success_event.wait()
            self.success_event.clear()
            if not self.success:
                continue
            #image = self.tracker.handsFinder(image)
            #lmList = self.tracker.positionFinder(image)
            image, centers = self.tracker.draw_bounding_box(self.image)
            cv2.imshow("Video", image)
            cv2.waitKey(1)
            # print(centers)
            if len(centers) > 0:
                x, y = centers[0][0], centers[0][1]
                #if self.last_x is None or self.last_y is None:
                    #self.last_x = x  # Initialize last_x and with the first x value
                    #self.last_y = y
                if not (x == 160): #need to add error handling for going out of degree range for j1
                    #self.j1 -= 0.02 * (x - 150)
                    self.j1 -= 0.03 * (x - 160) -  0.04 * (self.last_x - x)
                    #self.j1 -= -0.1 * ((self.last_x - 150) - (x - 150)) #test
                if not (y == 120): #need to add error handling for going out of degree range for j4
                    #self.j4 -= 0.02 * (y - 107)
                    self.j4 -= 0.03 * (y - 120) - 0.04 * (self.last_y - y)
                # Send joint angles to MyCobot
                self.mc.send_angles([self.j1, self.j2, self.j3, self.j4, 0, -135], 100)      
                # Update last x and y
                #self.j2 += 1
                #self.j3 -= 1
                self.last_x, self.last_y = x, y
    
def main(): #this function is outdated and will not work in current state
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
