# Co-Lab Robotics Team! Robot API project
This API allows users to interact with the robot without using any python code. By only sending an HTTP request, the user will be able to use the different preprogram functions. The following document contains the necessary information about the API developed.

# How to send an HTTP request to the robot
To send an HTTP request to the robot it is recommended to use an API developed platform like Insomnia since some of the API endpoints take a JSON format as its input.
To use the API, all of the endpoints have the following format, where <ROUTE> is the route the you are using:
```bash
http://10.194.72.227:5000/<ROUTE>
```  

# About the routes

## /default
This route sets the robot to the default position and sets the color of the LED white. No parameters are necessary.


## /lights
Allows the user to set an specific RGB color to the LED. This route taskes as a parameter the rgb colors. For example:
```bash
http://10.194.72.227:5000/lights?r=<RED COLOR>?g=<GREEN COLOR>?b=<Blue color>
``` 

## /show
Displays a RGB color gradient on the LED. This process takes about 20 seconds to complete. 

## /dance
Goes through a twerking dance, designed by Danai Adkisson, where the robot moves down and goes crazy. 

## /getAngles
Returns a JSON with the angles for all the 6 joints. 

## /getCoordinates
Returns a JSON with the coordinates for where the head of the robot is.

## /sendAngles
POST request, moves robot to desired position, takes in a JSON file in the following format: 
```bash
{
    "angles": [0, 0, 0, 0, 0, 0],
    "speed": 70
}
```
Angles refers to each of the 6 angles. Speed is the speed of the movement, with 50 being no speed and <50 being movement counter-clockwise, and >50 being movement clockwise. 

## /sendCoordinates
POST request, moves robot's head to desired position, takes in a JSON file in the following format: 
```bash 
{
    "coords": [0, 0, 0, 0, 0, 0],
    "speed": 70,
    "mode": 0
}
```
Angles refers to each of the 6 angles. Speed is the speed of the movement, with 50 being no speed and <50 being movement counter-clockwise, and >50 being movement clockwise. Mode refers to how the robot moves to that coordinate and can either be 0 or 1, with 1 being linear and 0 being angular (non-linear). 

## /release
Releases all servos of the robot, allowing for the robot to be easily manipulated into any position.
