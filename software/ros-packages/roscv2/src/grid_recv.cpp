#include "obstacle_detect.hpp"

int main(int argc, char *argv[]) {
	/* Init ROS */
	ros::init(argc, argv, "grid_recv");
	ros::NodeHandle n;
	ROS_INFO("grid_recv started");

	ros::Publisher pub = n.advertise<roscv2::Grid>("obstacle_grid", 100);
	ros::Rate rate(10);

	/* Spin */
	while (ros::ok()) {
		ros::spinOnce();
	}

	return 0;
}

