#include "obstacle_detect.hpp"

#define ROVER_WIDTH (1.25)
#define MAX_OBS_DIST (3.0)

void grid_callback(const roscv2::Grid& msg);
bool forward_obstacle(const Grid& grid);
void print_grid(const Grid& grid);
