#/usr/bin/bash

#load ros
source /opt/ros/groovy/setup.bash
#add rover_ws to path so we dont have to source it everytime
source /home/rover/ros/rover_ws/devel/setup.bash

roscore &
#start motor controller
sudo bash ~/rover_ws/src/roscv/scripts/motor_contoller.bash &

#start socket2ros and better_sender
rosrun roscv socket2ros.py &
rosrun roscv better_sender.py &
