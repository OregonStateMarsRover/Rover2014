#include "obstacle_detect.hpp"
#include <map>


//Options
#define MOVE

#ifndef PI
#define PI (3.1415)
#endif

#define ROVER_WIDTH (1.25)
#define MAX_OBS_DIST (5.0)
#define ARC_RAD (PI/8)

void grid_callback(const roscv2::Grid& msg);
bool forward_obstacle(const Grid& grid);
void score_directions(const Grid& grid, std::map<int, float>& scores);
void print_grid(const Grid& grid);
void move(bool blocked, std::map<int, float>& scores);
static void catch_sig(int sig);
