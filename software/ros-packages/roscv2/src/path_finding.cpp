#include "path_finding.hpp"

int main(int argc, char **argv) {

    ros::init(argc, argv, "path_finding");
    ros::NodeHandle pf;
    ROS_INFO("Path finding node online");

    ros::Subscriber sub = pf.subscribe("/obstacle_grid", 10, grid_callback);

	while (1) {
		ros::spinOnce();
	}
}

void grid_callback(const roscv2::Grid& msg) {
	Grid grid = Grid::from_msg(msg);
	print_grid(grid);
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
