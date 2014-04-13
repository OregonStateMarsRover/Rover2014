#include "path_finding.hpp"
#include <cmath>
#include <string>

static ros::Publisher motor_pub;

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_finding");
    ros::NodeHandle pf;
    ROS_INFO("Path finding node online");

    ros::Subscriber sub = pf.subscribe("/obstacle_grid", 10, grid_callback);
	motor_pub = pf.advertise<std_msgs::String>("/motor_command", 100);

	while (1) {
		ros::spinOnce();
	}
}

void grid_callback(const roscv2::Grid& msg) {
	Grid grid = Grid::from_msg(msg);
	print_grid(grid);

	bool blocked = forward_obstacle(grid);
	move(blocked);
	ROS_INFO("The rover is %s blocked!", blocked ? "" : "not");
}

bool forward_obstacle(const Grid& grid) {
	float square_width = grid.real_width() / (float)grid.width();
	float square_height = grid.real_height() / (float)grid.height();

	int rover_w_sq = (int)ceil((ROVER_WIDTH/2.0) / square_width);
	int max_dist_sq = (int)ceil(MAX_OBS_DIST / square_height);

	int bound_l = (int)floor(((float)grid.width()-1.0)/2.0)-rover_w_sq;
	int bound_r = (int)ceil(((float)grid.width()-1.0)/2.0)+rover_w_sq;
	ROS_INFO("Bounds: %d - %d", bound_l, bound_r);
	ROS_INFO("Dist: %d", max_dist_sq);

	for (int j = 0; j < max_dist_sq; j++) {
		for (int i = bound_l; i <= bound_r; i++) {
			if (grid[i][j] != 0) return true;
		}
	}
	return false;
}

void print_grid(const Grid& grid) {
	for (int j = 0; j < grid.height(); j++) {
		for (int i = 0; i < grid.width(); i++) {
			printf("%c ", grid[i][j] == 0 ? '-' : '#');
		}
		printf("\n");
	}
	printf("\n\n\n");
}

void move(bool blocked) {
	std::stringstream fss;
	fss << "flush";
	std_msgs::String flush_msg;
	flush_msg.data = fss.str();
	motor_pub.publish(flush_msg);
	if (!blocked) {
		std_msgs::String move_msg;
		std::stringstream mss;
		mss << "f1";
		move_msg.data = mss.str();
		motor_pub.publish(move_msg);
	}
}
