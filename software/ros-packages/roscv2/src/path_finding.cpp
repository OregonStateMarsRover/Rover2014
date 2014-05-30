#include "path_finding.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <signal.h>

static ros::Publisher motor_pub;
static ros::Publisher blocked_pub;
static float g_distance = MAX_OBS_DIST;
static float g_good_thresh = 0.3f;

static void catch_sig(int sig) {
    ROS_INFO("Caught signal-- hanging up");
	if (sig == SIGINT || sig == SIGQUIT || sig == SIGHUP) {
		std::stringstream fss;
		fss << "flush";
		std_msgs::String flush_msg;
		flush_msg.data = fss.str();
		motor_pub.publish(flush_msg);
	}

	exit(0);
}


int main(int argc, char **argv) {
	char *dval;
	int c;
	while ((c = getopt(argc, argv, "d:")) != -1) {
		switch (c) {
		case 'd':
			dval = optarg;
			g_distance = atof(dval);
			ROS_INFO("d is %f.", g_distance);
			break;
		case '?':
			if (optopt = 'd') {
				ROS_ERROR("d must have a value.");
			} else {
				ROS_ERROR("Unknown option %c.", optopt);
			}
			exit(0);
		}
	}

    ros::init(argc, argv, "path_finding");
    ros::NodeHandle pf;
    ROS_INFO("Path finding node online");

    ros::Subscriber sub = pf.subscribe("/obstacle_grid", 10, grid_callback);
	motor_pub = pf.advertise<std_msgs::String>("/motor_command", 100);
	blocked_pub = pf.advertise<std_msgs::Int32>("/blocked", 10);

	//TODO: FUNCTION ME
	std::stringstream fss;
	fss << "rover";
	std_msgs::String flush_msg;
	flush_msg.data = fss.str();
	motor_pub.publish(flush_msg);

	ros::spin();
}

void print_scores(std::map<int, float>& scores) {
	for (std::map<int,float>::iterator i = scores.begin(); 
	     i != scores.end(); i++) {
		printf("%d\t%f\n", i->first, i->second);
	}
}

void grid_callback(const roscv2::Grid& msg) {
	std_msgs::Int32 blocked_msg;
	Grid grid = Grid::from_msg(msg);

	float good = msg.pct_good;

	if (good < g_good_thresh * THRESH) {
		ROS_INFO("Not enough data in grid. - %f/%f", good, g_good_thresh);
		g_good_thresh = g_good_thresh * THRESH_DECAY;
		blocked_msg.data = -1;
	} else {
		g_good_thresh += (good - g_good_thresh) * THRESH_GROWTH;

		//print_grid(grid);

		std::map<int, float> scores;
		bool blocked = forward_obstacle(grid);
		blocked_msg.data = blocked ? 1 : 0;
		ROS_INFO("The rover is %s blocked!", blocked ? "" : "not");

		score_directions(grid, scores);
		move(blocked, scores);
		print_scores(scores);
	}

	blocked_pub.publish(blocked_msg);
}

bool forward_obstacle(const Grid& grid) {
	float square_width = grid.real_width() / (float)grid.width();
	float square_height = grid.real_height() / (float)grid.height();

	int rover_w_sq = (int)ceil((ROVER_WIDTH/2.0) / square_width);
	int max_dist_sq = (int)ceil(g_distance / square_height);

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

			int b;
			float b_f = angle/ARC_RAD;
			if (b_f < 0) {
				b = (int)floor(b_f);
			} else {
				b = (int)ceil(b_f);
			}

			float score = 1.0 / (dist*dist*dist); //TODO
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
	static int turn_direction = FORWARD;
	std::stringstream fss;
	fss << "flush";
	std_msgs::String flush_msg;
	flush_msg.data = fss.str();
    //motor_pub.publish(flush_msg);

	std_msgs::String move_msg;
	std::stringstream mss;

	if (!blocked) {
		turn_direction = FORWARD;
		mss << "f10";
		move_msg.data = mss.str();
	} else {
		if (turn_direction == FORWARD) {
			float left_score, right_score;
			std::map<int, float>::iterator it;
			for (it = scores.begin(); it != scores.end(); ++it) {
				if (it->first < 0) {
					left_score += it->second;
				} else {
					right_score += it->second;
				}
			}

			if (left_score < right_score) {
				turn_direction = LEFT;
			} else {
				turn_direction = RIGHT;
			}
		}

		switch (turn_direction) {
			case LEFT:
				ROS_INFO("Moving left");
				mss << "r340";
				move_msg.data = mss.str();
				break;
			case RIGHT:
				ROS_INFO("Moving right");
				mss << "r20";
				move_msg.data = mss.str();
				break;
		}

	}
#ifdef MOVE
	motor_pub.publish(move_msg);
#endif
}
