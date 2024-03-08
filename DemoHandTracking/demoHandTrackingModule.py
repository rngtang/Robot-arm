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
from concurrent.futures import ThreadPoolExecutor


from mediapipe.tasks import python
from mediapipe.tasks.python import vision

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


class HandTrackingModule:
    def __init__(self):
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 30, -30, 0, 0, -135], 40)
        self.cap = cv2.VideoCapture(0)
        #lower res means faster tracking
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # Disable auto exposure
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -200)  # lowering exposure and brightness helps the camera focus on the hand in bright settings better
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, -50)
        self.tracker = handTracker()
        self.j1, self.j2, self.j3, self.j4 = 0, 30, -30, 0
        self.j2_ema, self.j3_ema = 30, -30
        self.last_x, self.last_y = 160, 120
        self.timestamp = 0
        self.success = False
        self.running = True
        self.image = None
        self.success_event = threading.Event()
        self.prevGesture = None
        self.multiplier = 1
        # self.iter = 0

    def start(self):
        # print("test1")
        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=self.gestureLiveStreamTracking, daemon=True).start()
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

    def gestureLiveStreamTracking(self):
        model_file = open('gesture_recognizer.task', "rb")
        model_data = model_file.read()
        model_file.close()
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = GestureRecognizerOptions(
        base_options=python.BaseOptions(model_asset_buffer=model_data),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.__result_callback,
            min_tracking_confidence=0.1)
        recognizer = GestureRecognizer.create_from_options(options)

        while self.running:
            self.success_event.wait()
            self.success_event.clear()
            if not self.success:
                continue
            frame = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            recognizer.recognize_async(mp_image, self.timestamp)
            self.timestamp += 1

    def __result_callback(self, result, output_image, timestamp_ms):
        if len(result.gestures) > 0:
            gesture = result.gestures[0][0].category_name
            if gesture == "Thumb_Up" and self.j2 > -130:
                if self.prevGesture == "Thumb_Up":
                    if self.multiplier < 5:
                        # print("test1")
                        self.multiplier += 1
                else:
                    self.multiplier = 1
                self.j2 -= 1 * self.multiplier
                self.j3 += 1 * self.multiplier
                self.prevGesture = "Thumb_Up"
            elif gesture == "Pointing_Up" and self.j2 < 130:
                if self.prevGesture == "Pointing_Up":
                    if self.multiplier < 5:
                        # print("test2")
                        self.multiplier += 1
                else:
                    self.multiplier = 1
                self.j2 += 1 * self.multiplier
                self.j3 -= 1 * self.multiplier
                self.prevGesture = "Pointing_Up"
            else:
                self.multiplier = 1
                self.prevGesture = "None"
        else:
            self.multiplier = 1
            self.prevGesture = "None"
        #print(self.multiplier)

    def control_loop(self):
        j2_ema, j3_ema = self.j2, self.j3 #exponential moving average for smoother movement
        alpha = 0.1 # Smoothing factor for EMA

        while self.running:
            if not self.success:
                continue

            image, centers = self.tracker.draw_bounding_box(self.image)
            #cv2.imshow("Video", image)
            #cv2.waitKey(1)

            if len(centers) > 0:
                x, y = centers[0][0], centers[0][1]
                if not (x == 160):
                    j1_delta = 0.03 * (x - 160) -  0.04 * (self.last_x - x)
                    j1_new = self.j1 - j1_delta
                    if (j1_delta < 0 and j1_new < 160) or (j1_delta > 0 and j1_new > -160):
                        self.j1 = j1_new
                if not (y == 120):
                    j4_delta = 0.03 * (y - 120) - 0.04 * (self.last_y - y)
                    j4_new = self.j4 - j4_delta
                    if (j4_delta < 0 and j4_new < 90) or (j4_delta > 0 and j4_new > -90): 
                        self.j4 = j4_new
                # Apply EMA to smooth j2 and j3 movements
                j2_ema_new = alpha * self.j2 + (1 - alpha) * j2_ema
                j2_delta = j2_ema_new - j2_ema #track wheather going forward or backward
                #print(self.prevGesture)
                # print("----")
                # print(j2_ema)
                # print(j2_delta)
                # print(j2_ema_new)
                # print("----")
                #should try and fix the lag when switching from gesture to gesture
                if (self.prevGesture == "Thumb_Up" and j2_delta < 0 and j2_ema_new > -90):
                    j2_ema = j2_ema_new #update join movement
                    j3_ema = alpha * self.j3 + (1 - alpha) * j3_ema
                    # print(self.multiplier)
                elif (self.prevGesture == "Thumb_Up" and j2_delta > 0 and j2_ema - 2 > -90):
                    self.j2 = j2_ema
                    j2_ema -= 1 #update join movement
                    self.j3 = j3_ema
                    j3_ema += 1
                    # print("test1")
                elif (self.prevGesture == "Pointing_Up" and j2_delta > 0 and j2_ema_new < 90):
                    j2_ema = j2_ema_new #update join movement
                    j3_ema = alpha * self.j3 + (1 - alpha) * j3_ema
                    # print(self.multiplier)
                elif (self.prevGesture == "Pointing_Up" and j2_delta < 0 and j2_ema + 2 < 90):
                    self.j2 = j2_ema
                    j2_ema += 1 #update join movement
                    self.j3 = j3_ema
                    j3_ema -= 1
                    # print("test2")
                # Send joint angles to MyCobot
                self.mc.send_angles([self.j1, j2_ema, j3_ema, self.j4, 0, -135], 100)      

                self.last_x, self.last_y = x, y

if __name__ == "__main__":
    HandTrackingModule = HandTrackingModule()
    HandTrackingModule.start()
    input("Press Enter to stop...")
    HandTrackingModule.stop()
