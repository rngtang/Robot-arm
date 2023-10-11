# Co-Lab Robotics Team!

This project allows a student to play Tic-Tac-Toe on an LCD touchscreen against our MyCobot robotic arm. This setup uses two Raspberry Pis (one for the LCD, one for the robotic arm) as well as Python to run the game and ROS to control the movement of the arm.

To play the game, one must run applications on both the robot and the LCD. 

For the LCD: 
*The LCD is touchscreen, but can also be connected through RealVNC for easier access. More information about setting up RealVNC can be found below. The 

# Working with the LCD
You'll need to go to the Raspberry Pi station and connect a mouse and keyboard if working on the LCD, or you can also connect to HDMI as well to work on a bigger screen. 

To switch between the LCD and HDMI views, use these commands in the pi terminal.

```bash
cd LCD-show/
sudo ./LCD35-show
sudo ./LCD-hdmi

```

## Setting up Remote Server

Get RealVNC Viewer for your laptop, and then connect to robot with using the info below. You do not need to sign into VNC Viewer.

* IP: 10.197.171.134
* Password: Elephant

## Setting up SSH

In RealVNC, go to terminal (linux) and type command:

```bash
ssh er@10.197.171.134
```

If this doesn't work, change the IP to the IP you have in RealVNC for the robot server (because idk why but the IP might be different for everyone).

After typing command, enter password (from remote server setup).

Then, go to VSCode on your personal laptop and make sure you have the remote server extension installed. Open a new VSCode project and then choose the "SSH connection from another host". Enter the host:

```bash
er@10.197.171.134
```

Then, enter password.
 
And now you should be done! You can make a test file in Desktop such as "touch hiIwashere.txt" in your VSCode, and check to see if it's in the robot's Desktop in RealVNC.

### Optimizing the SSH process

These are a few steps you can do to optimize the SSH process into the robot, so that you don't have to manually enter the IP every time.

Open a new VS Code window, then click on the blue "><" button. Choose "Connect to Host..." -> "Configure SSH Hosts..." -> "Users/<your_username>/.ssh/config". Add the following lines to the file:

```
Host <your_chosen_hostname>
  HostName 10.197.171.134
  User er
```

Replace <your_chosen_hostname> with your chosen hostname (for example, "mycobot"), then save the file. The next time you SSH into the robot in the terminal, instead of typing in the IP, you can just run

```bash
ssh <your_chosen_hostname>
```

When you SSH into the robot using VS Code, after choosing "Connect to Host...", <your_chosen_hostname> will already be listed as an option, so you don't have to manually type in the IP.

## Running python code

Make sure to always include these lines as setup.

```python
from pymycobot.mycobot import MyCobot

from pymycobot.genre import Angle

from pymycobot import PI_PORT, PI_BAUD

import time
```

This is how you make the robot object, include this line too:  

```python
mc = MyCobot("/dev/ttyAMA0", 1000000)
```

For an example code, run test.py on Desktop
