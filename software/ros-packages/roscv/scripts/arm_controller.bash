#/usr/bin/bash
source /opt/ros/groovy/setup.bash
#add rover_ws to path so we dont have to source it everytime
source /home/dancecommander/rover_ws/devel/setup.bash

rosrun roscv arm_controller.py

