#/usr/bin/bash

#load ros
source /opt/ros/groovy/setup.bash
#add rover_ws to path so we dont have to source it everytime
source /home/rover/ros/rover_ws/devel/setup.bash

#now create a named screen session
screen -d -m -S rover
#start motor controller
screen -S rover -X exec rosrun roscv rmotors.py

#start socket2ros and better_sender
screen -S rover -p 1 -X exec rosrun roscv socket2ros.py
screen -S rover -p 2 -x exec rosrun roscv better_sender.py
