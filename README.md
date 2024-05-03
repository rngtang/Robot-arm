# Co-Lab Robotics Team! Tic-Tac-Toe Project

This project allows a student to play Tic-Tac-Toe on an LCD touchscreen against our MyCobot robotic arm. This setup uses two Raspberry Pis (one for the LCD, one for the robotic arm) as well as Python to run the game and ROS to control the movement of the arm.

# Playing the Game
To play the game, one must run applications on both the robot and the LCD. These applications are hosted on `~/robotic-arm/TicTacToe`.

**For the LCD:**
* **Instructions:** With screen/RealVNC: From Desktop, click on `ssh_home.sh`. Choose either 'Execute' or Execute in Terminal'. From here, you're set! Move onto the robot.
* **Code:** The LCD is meant to host the GUI of the game and do the mathematical calculation for finding the robot's best move based on the current board. It contains two important files, `ttt.py` and `game.py`. 
  * `ttt.py` calculates the best move for the robot and sets up a Flask request to send the best move over to the robot (port 5000). 
  * `game.py` runs the Tic-Tac-Toe GUI and runs a while loop that continuously updates the student's move and calls on `ttt.py` to send the best move request.
  * The LCD is touchscreen, but can also be connected through RealVNC for easier control. More information about setting up RealVNC can be found below. 

**For the Robot:**
* **Instructions:** You will need two terminals, one to run ROS, and one to run the robot. 
* In one terminal, run these following commands: 
```bash
cd ~/robotic-arm/TicTacToe/
./launch_controls.sh
```
* In the other, run this: 
```bash
python ~/robotic-arm/TicTacToe/api/app.py
```
* **Code:** The robot only uses ROS and the `app.py` file. 
  * ROS is called upon through the `launch_controls.sh` shell script. To make the movement of the robot more precise, we made our own `controls.py` file in the robotic arm that allows one to control the arm using ROS instead of Python. For more information on how to run the arm using Python instead, go down to the 'Running Python for Arm Movement' section.
  * `app.py` receives the robot's best move from `ttt.py` (from the LCD) through the Flask endpoint. It then uses ROS (set up by `launch_controls.sh`) to move the robotic arm to that spot on the LCD touchscreen.


# Launching hand and gesture tracking through Robot Operating System (ROS)

Make sure the camera flange is connected to the robot. In a terminal, after you SSH into the robot, run
```bash
source home/er/launch.sh hand
```

## Using Foxglove Studio
Foxglove Studio is a very useful software that allows us to visualize ROS data in real time using interactive visualizations in customizable layouts. We can easily use Foxglove Studio to build Graphical User Interface (GUIs) to interact with the robot and understand what the robot system is doing.

1. Make sure you have [Foxglove Studio](https://foxglove.dev/download]) installed in your local environment.
2. Launch hand and gesture tracking through ROS. (see above)
3. In another terminal, after you SSH into the robot, run
```bash
source ~/launch.sh foxglove
```
  to launch the Foxglove websocket on the robot, which will establish the connection to your local Foxglove software.

4. Open Foxglove Studios on your computer (not on the robot), click on "Open Connection", and enter `ws://10.197.94.158:8765` for the WebSocket URL. Click "Open". 

In Foxglove Studio, you can then click on the "Add panel" symbol and choose "Image" for `Image` and `CompressedImage` messages and "Raw Message" for other types of ROS messages. Once a new panel is opened, enter the name of the topic to subscribe to.

## Other ROS terminals
If you only want to launch a terminal with the ROS catkin workspace already set up, after you SSH into the robot, run
```bash
source ~/launch.sh foxglove
```



# Setup

## Remote Server Setup
For the robot (`Ubuntu`): 
* IP: 10.194.72.227
* Password: Elephant
* To SSH into the robot from your local terminal: 
  ```bash
  ssh ubuntu@10.194.72.227
  ```

For the LCD (`swipe`): 
* IP: 10.194.200.37
* Password: Elephant
* To SSH into the robot from your local terminal: 
  ```bash
  ssh swipe@10.194.200.37
  ```

## Working with the LCD
If you need to use the LCD for more than just playing the game, there are three ways to connect onto the LCD's Raspberry Pi: 
1. Use RealVNC to remotely control the Raspberry Pi's server. The IP information is above ('Remove Server Setup'). This method is easiest. 
2. SSH into the LCD through your terminal. Those steps are also above.
3. Use the Raspberry Pi station at the Co-Lab and connect with a mouse and keyboard if working on the LCD. You can also connect to HDMI instead to work on a bigger screen. 
  * To switch between the LCD and HDMI views, use these commands in the pi terminal.
  * ```bash 
    cd LCD-show/
    sudo ./LCD35-show
    sudo ./LCD-hdmi
    ```

## Setting up SSH in VSCode
This example is for if you want to edit VSCode on your own device but still have it be SSH-ed into the robot. To do this for the LCD, just replace the IP of the robot with the IP of the LCD. 

In RealVNC, go to terminal (linux) and type command:

```bash
ssh -XY -t -p 5422 colab_developer@10.197.94.158
```

After typing command, enter password (from 'Remove Server Setup').

Then, go to VSCode on your personal laptop and make sure you have the remote server extension installed. Open a new VSCode project and then choose the "SSH connection from another host". Enter the host:

```bash
colab_developer@10.197.94.158
```
Then, enter password.


## Optimizing the SSH process
These are a few steps you can do to optimize the SSH process into the robot, so that you don't have to manually enter the IP every time.

Open a new VS Code window, then click on the blue "><" button. Choose "Connect to Host..." -> "Configure SSH Hosts..." -> "Users/<your_username>/.ssh/config". Add the following lines to the file:

```
Host <your_chosen_hostname>
  HostName 10.197.94.158
  User colab_developer
  Port 5422
```

Replace `<your_chosen_hostname>` with your chosen hostname (for example, "mycobot"), then save the file. The next time you SSH into the robot in the terminal, instead of typing in the IP, you can just run

```bash
ssh <your_chosen_hostname>
```

When you SSH into the robot using VS Code, after choosing "Connect to Host...", `<your_chosen_hostname>` will already be listed as an option, so you don't have to manually type in the IP.

## Running Python for Arm Movement
These commands are for if you want to control the movement of the robotic arm through Python instead of ROS. These commands are not used in our Tic-Tac-Toe game, they're just here for reference.

```python
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time
```

```python
mc = MyCobot("/dev/ttyAMA0", 1000000)
```

For an example code, run test.py on Desktop
