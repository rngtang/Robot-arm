# Co-Lab Robotics Team! Robot API project
This API allows users to interact with the robot without using any python code. By just sending an HTTP request, the user will be able to use different programmed functions. The following document contains the necessary information about the developed API.

## Starting the Server
First, make sure the robot is connected to a power outlet. Then, turn on the robot using the **red button** at the bottom. You may have to wait approximately 30s before proceeding to the next step.

# Remote Server Setup
Now, your next task is to establish an SSH connection with the robot. To do so, you will need to open a **Linux/Unix/WSL terminal** and run the following command:

```bash
  ssh ubuntu@10.194.29.175
```

When prompted, enter the following password:
```bash
  Elephant
```

# How to run the API
By now, you should have turned on and established an SSH connection with the robot. 
The next step is running the API. First, enter the local git branch by running the following command:
```bash
cd robotic-arm/
```

In order to run the API correctly, confirm you are on branch **build_apis_production** by using the command:
```bash
git branch
```
You should see a star next to the branch **build_apis_production**. 

If you are not on this branch, run
```bash
git checkout build_apis_production
```
Now, you should be able to run the API.
Run the following command
```bash
./runApi.sh
```

# How to send an HTTP request to the robot
It is recommended to use an API developed platform like Insomnia to send an HTTP request to the robot since some of the API endpoints take a JSON format as its input. If not using one of these platforms, then you should use your **browser**.
To use the API, all of the endpoints have the following format, where <ROUTE> is the route the you are using:
```bash
http://10.194.29.175:5000/<ROUTE>
```  

# About the routes

The following documentation contains some information about the routes. If you would like to have detailed information about each route, such as responses or parameters, checkout the [SwaggerHub documentation](https://app.swaggerhub.com/apis/RAULNGTANG/robotAPI/1.0.0)

## GET /default
This route sets the robot to the default position and sets the color of the LED white. No parameters are necessary.

## GET /lights
Allows the user to set a specific RGB color to the LED. This route can take in RGB values (0-255) as parameters. For example:
```bash
http://10.194.29.175:5000/lights?r=<RED COLOR>&g=<GREEN COLOR>&b=<Blue color>
``` 
If no parameters are given, the default RGB values are (10, 10, 10), and no light will be displayed on the robot.

## GET /show
Displays a RGB color gradient on the LED. This process takes about 20 seconds to complete. 

## GET /dance
Goes through a twerking dance, designed by Danai Adkisson, where the robot gets down and goes crazy. 

## GET /getAngles
Returns a JSON with the angles for all the 6 joints. 

## GET /getCoordinates
Returns a JSON with the coordinates for where the head of the robot is. The robot uses a 6-value coordinate system: [x,y,z,rx,ry,rz]. 

## POST /sendAngles
Moves robot to desired position by taking in a JSON file body.

**Parameters**
* angles:  a list of degree value(List[float]), length 6 -160 ~ 160
* speed: 0 ~ 100

'angles' refers to each of the 6 joints. 'speed' is the speed of the movement, with 50 being no speed and <50 being movement counter-clockwise, and >50 being movement clockwise. 

The JSON file should have the following format:

```bash
{
    "angles": [0, 0, 0, 0, 0, 0],
    "speed": 70
}
```

## POST /sendCoordinates
Moves robot's head to desired position by taking in a JSON file body.

**Parameters:**
* coords: a list of coords value(List[float]), length 6.
* speed: (int) 0 ~ 100
* mode: (int): 0 - angular, 1 - linear

'angles' refers to each of the 6 angles. 'speed' is the speed of the movement, with 50 being no speed and <50 being movement counter-clockwise, and >50 being movement clockwise. 'mode' refers to how the robot moves to that coordinate and can either be 0 or 1, with 1 being linear and 0 being angular (non-linear). 

The JSON file should have the following format:

```bash 
{
    "coords": [0, 0, 0, 0, 0, 0],
    "speed": 70,
    "mode": 0
}
```

## GET /release
Releases all servos of the robot, allowing for the robot to be easily manipulated into any position.
