#include "obstacle_detect.hpp"

int main(int argc, char *argv[]) {
	/* Init ROS */
	ros::init(argc, argv, "grid_pub");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<roscv2::Grid>("obstacle_grid", 100);
	ros::Rate rate(10);

	ROS_INFO("grid_pub started");

	Grid g = Grid(10,10,40.0,40.0);
	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			g[i][j] = (i+j)%2;
		}
	}

	/* Spin */
	while (ros::ok()) {
		roscv2::Grid msg;
		g.populate_msg(msg);
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

