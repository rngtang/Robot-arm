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
This route sets the robot to the default position and sets the color of the LED white. Currently doesn't work, testing with Controls.

## /lights

## /show

## /dance
