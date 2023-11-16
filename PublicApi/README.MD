# Co-Lab Robotics Team! Robot API project
This API allows users to interact with the robot without using any python code. By just sending an HTTP request, the user will be able to use different programmed functions. The following document contains the necessary information about the developed API.

<!-- # Testing with gunicorn 
```
gunicorn -b 10.194.72.227:5000 main.wsgi:app
gunicorn -c gunicorn_config.py main:app
``` -->
# Remote Server Setup
For the robot (`Ubuntu`): 
* IP: 10.194.29.175
* Password: Elephant
* To SSH into the robot from your local terminal: 
```bash
  ssh ubuntu@10.194.29.175
```

# How to run the API
First, to run the API, you will need to cd into the /robotic-arm directory of the robot.
```bash
cd robotic-arm/
```
The run the following command
```bash
./runApi.sh
```

# How to send an HTTP request to the robot
It is recommended to use an API developed platform like Insomnia to send an HTTP request to the robot since some of the API endpoints take a JSON format as its input.
To use the API, all of the endpoints have the following format, where <ROUTE> is the route the you are using:
```bash
http://10.194.29.175:5000/<ROUTE>
```  

# About the routes

## /default
GET request. This route sets the robot to the default position and sets the color of the LED white. No parameters are necessary.

## /lights
GET request. Allows the user to set an specific RGB color to the LED. This route can take in RGB values (0-255) as parameters. For example:
```bash
http://10.194.29.175:5000/lights?r=<RED COLOR>&g=<GREEN COLOR>&b=<Blue color>
``` 
If no parameters are given, the default RGB values are (10, 10, 10), and no light will be displayed on the robot.

## /show
GET request. Displays a RGB color gradient on the LED. This process takes about 20 seconds to complete. 

## /dance
GET request. Goes through a twerking dance, designed by Danai Adkisson, where the robot gets down and goes crazy. 

## /getAngles
GET request. Returns a JSON with the angles for all the 6 joints. 

## /getCoordinates
GET request. Returns a JSON with the coordinates for where the head of the robot is. The robot uses a 6-value coordinate system: [x,y,z,rx,ry,rz]. 

## /sendAngles
POST request. Moves robot to desired position by taking in a JSON file in the following format: 
```bash
{
    "angles": [0, 0, 0, 0, 0, 0],
    "speed": 70
}
```
Parameters
* angles:  a list of degree value(List[float]), length 6 -160 ~ 160
* speed: 0 ~ 100

'angles' refers to each of the 6 joints. 'speed' is the speed of the movement, with 50 being no speed and <50 being movement counter-clockwise, and >50 being movement clockwise. 

## /sendCoordinates
POST request. Moves robot's head to desired position by taking in a JSON file in the following format: 
```bash 
{
    "coords": [0, 0, 0, 0, 0, 0],
    "speed": 70,
    "mode": 0
}
```
Parameters:
* coords: a list of coords value(List[float]), length 6.
* speed: (int) 0 ~ 100
* mode: (int): 0 - angular, 1 - linear

'angles' refers to each of the 6 angles. 'speed' is the speed of the movement, with 50 being no speed and <50 being movement counter-clockwise, and >50 being movement clockwise. 'mode' refers to how the robot moves to that coordinate and can either be 0 or 1, with 1 being linear and 0 being angular (non-linear). 

## /release
GET request. Releases all servos of the robot, allowing for the robot to be easily manipulated into any position.
