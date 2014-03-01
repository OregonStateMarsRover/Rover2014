#include "ros/ros.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "obstacle_detect");
	ros::NodeHandle n;
	
	ROS_INFO("obstacle_detect started");

	while (ros::ok()) {
		ros::spinOnce();
	}
	return 0;
}
