import cv2
import mediapipe as mp
import time
import random
import math
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord

import threading
from concurrent.futures import ThreadPoolExecutor

#Mediapipe library for hand landmarks and gesture tracking.
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

#Needed to activate/disactive the pin that powers suction pump.
import RPi.GPIO as GPIO 

#Handtracker class that uses mediapipe's hand landmarker tracking to draw bounding box around hand and find bounding box center for handtracking.
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
    draw_bounding_box(self, image) - used to draw handbounding box and return the center of the bounding box as well
    """

    def __init__(self, mode=False, maxHands=1, detectionCon=0.7, modelComplexity=0, trackCon=0.1):
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
        #Setting the modelComplexity to 0 makes the handtracking much faster.
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def draw_bounding_box(self, image):
        """
        Draws a bounding box around detected hands in the given image and finds the center of the bounding box.

        Args:
            image (numpy.ndarray): The image to draw the bounding box on.

        Returns:
            image: A tuple containing the modified image with bounding boxes drawn.
            centers: The centers of the bounding boxes.

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

class MyCobotHandTrackingClass:
    """
    Primary class that performs hand landmark and gesture detection and controls the robot arm. 
    It uses the camerafeed from the Elephant Robotics camerflange attached to the head of the robot, but can be perhaps be fitted with a cheaper web camera that you can attach to the head of the robot. 
    The class achieves hand tracking by making the robot continually center the hand in the camera feed. 
    Additionally, the class performs gesture tracking to move the robot closer or further and contract/extend the robot. 
    The code has been fitted for myCobot280, but perhaps can be adapted for the M5 version. 
    The main purpose is to track the user's hand and allow them to pick up objects with the robot using just their hand, a camera, and suction pump (Elephant Robotics suction pump v1 was used for this project, but v2 or other suction pump can perhaps be adapted).

    The class performs hand tracking using joints 1 and 4.
    Joint 1 is used for horizontal tracking of the hand.
    Joint 4 is used for veritcal tracking of the hand.

    Additionally, the class uses the thumbs up and thumbs down gesture from the user to move the robot closer and further using joints 2 and 3. This is meant for picking up objects using the suction pump, but can be used for more general purposes.
    Thumbs up moves the robot away from you, so towards the object you are trying to pick up.
    Thumbs down moves the robot toward you, so to pick up the object.

    Lastly, to assist with pick up objects, the robot also contracts and extends using a closed fist gesture. 
    To contract the robot, use a closed fist and move downward. 
    To extend the robot, use a closed fist and move upward.
    """
    def __init__(self):
        #Initialize MyCobot and set arm to default starting position.
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.mc.send_angles([0, 0, 0, 0, 0, -135], 20)

        #Initialize handTracker class which draws bounding box around hand and returns the center of the hand's bounding box.
        self.tracker = handTracker()

        #Set MyCobot joint angles.
        self.j1, self.j2, self.j3, self.j4 = 0, 0, 0, 0
        #Ema or exponential moving average makes rotation of joints 2 and 3 more smooth.
        self.j2_ema, self.j3_ema = 0, 0

        #Set the a default x and y position of hand to be in the center of the camera to be used in handtracking formula.
        self.last_x, self.last_y = 160, 120
        #Initialize camera_angle which is the angle of camera from horizontal axis.
        self.camera_angle = 0
        
        #To keep track of previous hand gesture
        self.prevGesture = None
        #A gesture multiplier that increases the longer you hold a specified gesture to speed up joint rotations.
        self.gestureMultiplier = 1

        #Initialize camera and camera settings
        self.cap = cv2.VideoCapture(0)
        #Lowering width and height makes tracking faster by effectively lowering the resolution of image and processing time needed per image.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        #Disabing autoexposure and lowering exposure and brightness allows camera to better focus on hand in moderate to high light environments.
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # Disable auto exposure
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -100) #lower exposure
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, -25) #lower brightness
        #If you experience issues with the lighting, play around with these camera settings. 
        #This is the best configuration I've found for moderate light environments.

        #For threading and concurrency
        self.success = False
        self.running = True
        self.image = None
        self.success_event = threading.Event()     
        self.timestamp = 0

        #For pump activation
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(1, GPIO.OUT)
        GPIO.output(1, 1) #Turn off pump by default
        self.pump_active = False


    def start(self):
        #Begin multithreading of camera_loop, gestureLiveStreamTracking, and control loop
        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=self.gestureLiveStreamTracking, daemon=True).start()
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop(self):
        #End multithreading
        self.running = False
    
    def camera_loop(self):
        #Camera loop to continously set camera image
        while self.running:
            success, image = self.cap.read()
            if not success:
                continue
            self.success_event.set()
            rotated_image = cv2.rotate(image, cv2.ROTATE_180)
            self.image = rotated_image
            self.success = success

    def gestureLiveStreamTracking(self):
        #Intailize Mediapipe hand gesture tracking model
        #Mediapipe does not offer model api for the gesture model such as hand landmarker model, so you must directly include the gesture model in the same folder.
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
            #Call gesture recognizer with asynchronous call to __result_callback 
            recognizer.recognize_async(mp_image, self.timestamp)
            self.timestamp += 1

    def __result_callback(self, result, output_image, timestamp_ms):
        #Initalizing pickingUpObjectMultiplier that is used to lower the speed of the robot moving foward when it detects someone is trying to pick up an object. 
        #It does this by using the angle between the camera's line of vision and the horizontal axis. 
        #This is to make it easier to control the robot when picking up an object.
        pickingUpObjectMultiplier = 1

        if len(result.gestures) > 0:
            #The recognized gesture that the model detects
            gesture = result.gestures[0][0].category_name
            
            #Adjusts j2:j3 ratio based on camera angle to allow for intuitive motion when picking up objects
            #The greater the angle between the camera's line of vision and the horizontal axis, the closer the object is that the user is trying to pick up so the j2:j3 ratio is greater and will be applied to j3 to rotate it at a greater rate than j2 to pick things up that are closer
            j2_j3_ratio = ((-1.18 * self.camera_angle) / 90)               

            #If angle between the camera's line of vision and the horizontal axis is greater than 55 degrees, but less than 125 degrees the speed of the robot approaching the object is reduced by 50% to allow for users to have more control when picking up objects
            if abs(self.camera_angle) > 55 and abs(self.camera_angle) < 125:
                pickingUpObjectMultiplier = 0.5
            else:
                pickingUpObjectMultiplier = 1

            if gesture == "Thumb_Up" and self.j2 > -130:
                #Set gesture multipler to increase the movement speed of robot the longer the user holds the thumbs up gesture using this gesture multipler
                if self.prevGesture == "Thumb_Up":
                    if self.gestureMultiplier < 3:
                        self.gestureMultiplier += 0.5
                else:
                    self.gestureMultiplier = 1

                #Rotate j2, taking into account the gesture multiplier and pickingUpObjectMultiplier
                self.j2 -= 1 * self.gestureMultiplier * pickingUpObjectMultiplier

                #If camera is pointing to something out of range
                if self.camera_angle >= -10 and self.j3 < 0:
                    #This extends arm when to try reaching the out of range object
                    j2_j3_ratio = max(1, j2_j3_ratio)
                    self.j3 += 1 * self.gestureMultiplier * (j2_j3_ratio) * pickingUpObjectMultiplier

                #If the object is in range, apply the j2:j3 ratio to j3 to move joints effectivley to approach object
                else:
                    j2_j3_ratio = max(0, j2_j3_ratio)
                    self.j3 -= 1 * self.gestureMultiplier * (j2_j3_ratio) * pickingUpObjectMultiplier

                self.prevGesture = "Thumb_Up"
        
            #Similar logic as the lines above for the thumbs down gesture. 
            #A bit less complex than thumbs up gesture logic since thumbs down is not used when a user to trying to pick up an object
            elif gesture == "Thumb_Down" and self.j2 < 130:
                if self.prevGesture == "Thumb_Down":
                    if self.gestureMultiplier < 3:
                        self.gestureMultiplier += 0.5
                else:
                    self.gestureMultiplier = 1
                self.j2 += self.gestureMultiplier
                self.j3 += self.gestureMultiplier * (j2_j3_ratio) * pickingUpObjectMultiplier
                self.prevGesture = "Thumb_Down"
            
            #If a closed fist gesture is detected
            #A closed fist getsure will be used to contract and extend robot arm.
            elif gesture == "Closed_Fist":
                self.prevGesture = "Closed_Fist"

            #Activates and deactivates pump using index finger pointed up gesture
            elif not (self.prevGesture == "Pointing_Up") and gesture == "Pointing_Up":
                if self.pump_active == False:
                    GPIO.output(1, 0) #Turn on pump
                else:
                    GPIO.output(1, 1) #Turn off pump
                self.pump_active = not self.pump_active
                self.prevGesture = "Pointing_Up"

            #If no gesture is detected reset gesture multiplier and set gesture to none
            else:
                self.gestureMultiplier = 1
                self.prevGesture = "None"

        #If no hand is detected reset gesture multiplier and set gesture to none
        else:
            self.gestureMultiplier = 1
            self.prevGesture = "None"

    def control_loop(self):
        #Exponential moving average for smoother movement. 
        #Exponential moving average weighs the current and last joint angles for smoother movement. 
        #Used for thumbs up and thumbs down movements only since those movements do not use hand coordinate information (which inherently makes movement less smooth)
        self.j2_ema, self.j3_ema = self.j2, self.j3
        # alpha is the smoothing factor for the exponential moving average. 
        #A lower alpha corresponds to weighing the last joint angle more which makes the movement smooth. 0.2 is the value I found to be the most stable and smooth.
        alpha = 0.2
        prevGesture = self.prevGesture
        #J1_multiplier used to slown down the horizontal tracking when the robot head is far away from the center vertial axis.
        #This makes it easier to control the movements when the arm is extended out greatly.
        j1_multiplier = 1

        while self.running:
            if not self.success:
                continue
            
            image, centers = self.tracker.draw_bounding_box(self.image) #Retrieve image and centers
            cv2.imshow("Video", image) # To show camera feed
            cv2.waitKey(1)
            #If you want to run the file from your own terminal, shh into the robot and comment out the two lines above. 
            #The camera feed will give errors if you are not using a terminal in the raspberry pi.

            if len(centers) > 0:
                x, y = centers[0][0], centers[0][1] #The user's hand center coordinates
                #The center of the camera is (160, 120)

                #If the hand is not horizontally centered, center it by rotating joint 1
                if not (x == 160):
                    #J1_delta uses a fine tuned formula for calculating how much to rotate joint 1 based on the x coordinate of the hand to horizontally center it. 
                    #It uses two parts - 
                    #1. the distance between the hand's x coordinate and the center x cordinate.
                    #2. the distance between the hand's last and current x coordinate. 
                    #The forumla is very accurate with minimal overshot for horizontally centering the hand.
                    j1_delta = 0.03 * (x - 160) -  0.04 * (self.last_x - x)
                    j1_delta = j1_multiplier * j1_delta

                    #If user is trying to pick up an object, lower the amount of rotation of joint 1 to make it easier to control. 
                    #Additionally, make the movement none inverted to also make it easier.
                    if self.camera_angle < -55 and self.camera_angle > -125:
                        j1_new = self.j1 + 0.5 * (j1_delta)

                    else:
                        j1_new = self.j1 - j1_delta
                    
                    #For out of bounds handling of joint 1 rotation
                    if (j1_delta < 0 and j1_new < 160) or (j1_delta > 0 and j1_new > -160):
                        self.j1 = j1_new

                #If the hand is not vertically centered, center it by rotating joint 4
                if not (y == 120):
                    #For a closed fist gesture, vertically center the hand more slowly. 
                    #This allows for the user to contract/extend the robot arm by moving their hand downward/upward without the robot camera centering on the hand too quickly
                    if self.prevGesture == "Closed_Fist":
                        #J4_delta uses a fine tuned formula for calculating how much to rotate joint 4 based on the y coordinate of the hand to vertically center it. 
                        #It uses two parts - 
                        #1. the distance between the hand's y coordinate and the center y cordinate.
                        #2. the distance between the hand's last and current y coordinate.
                        j4_delta = 0.03 * (y - 120) - 0.04 * (self.last_y - y)

                    else:
                        j4_delta = 0.04 * (y - 120) - 0.05 * (self.last_y - y)
                    j4_new = self.j4 + j4_delta
                    self.j4 = j4_new
                
                #If the user has a closed fist gesture, contract the robot arm when they move downward and extend the arm when they move upward.
                if self.prevGesture == "Closed_Fist" and (not (y == 120)):
                    #Use joints 2 and 3 to contract/extend the robot arm.
                    #Since we ARE using the hand coordinate information, we do not need the exponential moving average smoothing effect, so we modify both the ema joint value and current joint value to not affect the difference between the two
                    self.j2_ema += 0.03 * (y - 120) - 0.04 * (self.last_y - y)
                    self.j2 += 0.03 * (y - 120) - 0.04 * (self.last_y - y)
                    self.j3_ema -= 2 * (0.03 * (y - 120) - 0.04 * (self.last_y - y))
                    self.j3 -= 2 * (0.03 * (y - 120) - 0.04 * (self.last_y - y))

                # Apply EMA to smooth j2 and j3 movements
                j2_ema_new = alpha * self.j2 + (1 - alpha) * self.j2_ema
                j3_ema_new = alpha * self.j3 + (1 - alpha) * self.j3_ema
                j2_delta = j2_ema_new - self.j2_ema #track wheather going forward or backward
                
                #Logic that nullifies some odd behavior of the ema. Since the ema weighs the last joint angle, this causes the robot to slightly continue moving in a certain direction even though you stopped a certain gesture. 
                #It also causes other weird behavior such as lag or abrupt movements. This can make the thumbs up and thumbs downs movement hard to use. 
                #HOWEVER, this logic stops any odd behavior and allows for more intuative behavior of thumbs up and thumbs down movement.
                if (self.prevGesture == "Thumb_Up" and j2_delta > 0) or (self.prevGesture == "Thumb_Up" and prevGesture != "Thumb_Up"):
                    self.j2 = self.j2_ema
                    self.j3 = self.j3_ema
                elif (self.prevGesture == "Thumb_Down" and j2_delta < 0) or (self.prevGesture == "Thumb_Down" and prevGesture != "Thumb_Down"):
                    self.j2 = self.j2_ema
                    self.j3 = self.j3_ema
                elif (self.prevGesture == "Thumb_Up" and j2_delta < 0):
                    self.j2_ema = j2_ema_new
                    self.j3_ema = j3_ema_new
                elif (self.prevGesture == "Thumb_Down" and j2_delta > 0):
                    self.j2_ema = j2_ema_new
                    self.j3_ema = j3_ema_new
                
                #Calculates roughly how far the robot head is from the z axis (vertical center axis) using joints 2 and 3. 
                #Used angles instead of coords to calculate this beause get_cords slows the tracking down. 
                #Only calculate when the user uses a gesture that modifies joints 2 or 3 since these are the only ones used for the calculations.
                if self.prevGesture == "Thumb_Down" or self.prevGesture == "Thumb_Up" or self.prevGesture == "Closed_Fist":
                    value = (abs(self.j3) - abs(self.j2))
                    j1_multiplier = abs((-abs(self.j2) / 90) + 1)
                    j1_multiplier += 0.87 * (abs((( value / ( 90)) + 1))) #Using 0.87 because the ratio of length between the robot portion between above j2 and j3 vs the portion between j3 and j4 is 100:87
                    j1_multiplier = min(1.87, j1_multiplier)
                    j1_multiplier = (.27 * j1_multiplier) + 0.5

                #Angle between the line of vision of the camera and horizontal axis. 
                #Used to detect when the user is roughly pointing the camera toward the ground.
                self.camera_angle = self.j2_ema + self.j3_ema + self.j4

                prevGesture = self.prevGesture

                # Send joint angles to myCobot
                self.mc.send_angles([self.j1, self.j2_ema, self.j3_ema, self.j4, 0, -135], 100)

                self.last_x, self.last_y = x, y

if __name__ == "__main__":
    # Logic to call the MyCobotHandTrackingClass class to begin hand tracking
    controller = MyCobotHandTrackingClass()
    controller.start()
    input("Press Enter to stop...")
    controller.stop()