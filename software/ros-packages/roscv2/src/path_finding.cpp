#include "path_finding.hpp"
#include <std_msgs/String.h>
#include <cmath>

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

	std::map<int, float> scores;
	bool blocked = forward_obstacle(grid);
	score_directions(grid, scores);
	move(blocked, scores);
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

void score_directions(const Grid& grid, std::map<int, float>& scores) {
	float square_width = grid.real_width() / (float)grid.width();
	float square_height = grid.real_height() / (float)grid.height();
	float rover_pos_x = ((float)grid.width() - 1.0) / 2.0;
	float rover_pos_y = 0;
	for (int j = 0; j < grid.height(); j++) {
		for (int i = 0; i < grid.width(); i++) {
			if (grid[i][j] == 0) continue;
			float dx = (float)i - rover_pos_x;
			float dy = (float)j - rover_pos_y;
			float x_dist = dx * square_width;
			float y_dist = dy * square_height;
			float dist = sqrt(x_dist*x_dist + y_dist*y_dist);
			float angle = atan2(dx, dy);

			int b = (int)ceil(angle / ARC_RAD);

			float score = 1.0 / (dist*dist); //TODO
			if (scores.count(b) > 0) {
				scores[b] += score;
			} else {
				scores[b] = score;
			}
		}
	}
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

void move(bool blocked, std::map<int, float>& scores) {
#ifndef MOVE
	return;
#endif
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
