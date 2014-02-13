echo "This Starts The ROS stuff"&&
roscore &&
cd  ros/rover_ws/ &&
source devel/setup.bash &&
roslaunch roscv startSingleCam.launch
