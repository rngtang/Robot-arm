**NOTE**: If you notice that you cannot push and pull from the robot anymore, then the project access token might be expired. Please visit step 6 of the **Robot setup from scratch** section to create a new one. **Current access token expiring on December 1st, 2024**.

## Robot setup from scratch

**NOTE**: You will need a monitor, keyboard, and mouse to do this. Make sure the monitor is connected to the robot **before** you turn it on, otherwise it won't provide HDMI output.

1. To connect to DukeBlue: 
  - Click the wi-fi icon on the top right, you will see the robot is connected to it's own wi-fi
  - Press 'Disconnect'. Then you'll see other network names pop up. Select DukeBlue
  - Follow the instructions in this [ServiceNow article](https://duke.service-now.com/nav_to.do?uri=%2Fkb_view.do%3Fsys_kb_id%3Dad4f42681bb545103afd54a7624bcb85%26sysparm_rank%3D1%26sysparm_tsqueryId%3D318281348709b590a92dfa87dabb35bc) until Step 7
  - **NOTE**: You need to click the 'No CA Cert' checkbox or it won't let you complete the connection
  - If that fails, you will need to follow the instructions in the 'CA Certificate' subsection of Step 8
    - On your computer download the certificate. Then open your terminal and cd into the directory where the cert is
    - On the robot, temporarily connect to DukeOpen. Then open the terminal and type `ifconfig` to find your IP address
    - Back in your own terminal, inside of the directory with the cert, type `scp [CERT NAME] er@[ROBOT IP ADDRESS]:/.` This will copy the cert from your computer into the robot
    - Now, on the robot's Network Connections setup window, you can uncheck 'No CA Cert' and apply the key instead

2. Try connecting to the robot with the default username, port, and password:
  - Open the robot's terminal and type `ifconfig` to find your IP address
  - On your computer's terminal, type `ssh er@[ROBOT IP ADDRESS]`
    - The default password is *Elephant*

3. To change user password, click on the Menu in the top left then Administration > Users and Groups. Select your account and change your password

4. Change SSH port to not be the default 22
  - Open SSH configurations: `sudo nano /etc/ssh/sshd_config`
  - Scroll to where it says `#Port 22`
  - Create a new line under that one, and write `Port [YOUR PORT NUMBER]`
  - Save and exit
  - Restart the robot. **NOTE**: You can restart the robot through the Menu like every other computer, no need to switch it off and on
  - Try reconnecting with the command `ssh -p [YOUR PORT NUMBER] er@[ROBOT IP ADDRESS]`
  - 🤞

5. If you'd like to change the user ID to something other than er, see the **Change User ID** section

6. Try connecting to the robot using VNC with the default pasword (*Elephant*)

7. Change the default VNC password:
  - In the robot's terminal, type `vncpasswd`
  - Input and confirm the new password

8. Set up the project access token so that we can push and pull to the Robotic Arm GitLab repo
  - Go to the project on GitLab. Then on the left menu bar go to Settings > Access Tokens
  - Create a new token. **NOTE**: Access tokens must have an expiry date, and their maximum duration is one year. Please make sure to set this properly otherwise GitLab will automatically give it a lifespan of one month.
  - Give it the 'developer' role and the 'write-repository' permission
  - Once you click create, the key will show. **NOTE**: Make sure to copy it! This is the only time you will be shown the key
  - Things to note:
    - Project access keys create a bot user in your project. By going to the Manage > Members tab you can see the bot's username. The password is the key
    - You will be asked for the username and password every time you try to push to the repo. To avoid this, we can save the credentials locally on our machine using [git-credentials-store](https://git-scm.com/docs/git-credential-store#FILES). See step 8 for details

9. Use HTTP (not SSH) to clone the repo into the robot's Desktop directory. Make a test edit on this README file and then push. You will be prompted for the bot account's username and password

10. To save the username and password locally in our machine:
  - Create the file where they will be saved: `touch ~/.git-credentials`
  - Tell git to save the credentials next time we authenticate: `git config credential.helper store`
  - Make a new push. This time, after you enter the creds, they will be saved
  - To test that it worked, try to push again. This time you won't be prompted


## Change User ID

1. In the meny, go to Administration > Users and Groups
2. Create a new user called temp
3. In temp's user advanced user settings, add it to the sudo group
4. Click the meny icon and search Login Window. A window will pop up
5. Click the second tab of this window. At the bottom of that tab, you will find a field that says 'er'. This is the account that the bot automatically logs into upon boot
6. Delete 'er' and chage it to 'temp'. Then restart the robot. **NOTE**: You can restart the robot through the Menu like every other computer, no need to switch it off and on
7. Upon booting up, you will find yourself in temp's home directory instaed of er's. **NOTE**: It's in Chinese 😨! But don't fret! You don't need to understand anything to make this work
8. Open the terminal (you can search the term 'terminal' in the menu) and use the following command `sudo usermod -l colab_dev er` or whatever other user ID you'd want
9. Now, go back to Administration > Users and Groups (you can also type Users and Groups into the search bar)
10. Repeat step 6 this time changing 'temp' for the new user ID. Then restart the robot once again
11. Hopefully upon booting up this time you'll be back in our familiar desktop again!
12. Once again, go into Administration > Users and Groups and delete the temp user along with all it's files
13. On that same window, you can also change your user's username to match the new ID (username and user ID are two different things)
14. Voila!

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
ssh ubuntu@10.194.72.227
```

After typing command, enter password (from 'Remove Server Setup').

Then, go to VSCode on your personal laptop and make sure you have the remote server extension installed. Open a new VSCode project and then choose the "SSH connection from another host". Enter the host:

```bash
ubuntu@10.194.72.227
```
Then, enter password.


## Optimizing the SSH process
These are a few steps you can do to optimize the SSH process into the robot, so that you don't have to manually enter the IP every time.

Open a new VS Code window, then click on the blue "><" button. Choose "Connect to Host..." -> "Configure SSH Hosts..." -> "Users/<your_username>/.ssh/config". Add the following lines to the file:

```
Host <your_chosen_hostname>
  HostName 10.197.171.134
  User er
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
