#!/bin/bash

cd ../catkin_ws
catkin_make --only-pkg-with-deps mycobot_280
source devel/setup.bash

nohup roslaunch mycobot_280 controls.launch &
