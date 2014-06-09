#include "path_finding.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <signal.h>


static ros::Publisher motor_pub;
static ros::Publisher blocked_pub;
static float g_distance = MAX_OBS_DIST;
static float g_good_thresh = 0.3f;
static float g_angle; //TODO: TEMPORARY
static float g_goal_distance; //TODO: TEMPORARY
static float g_has_goal;
static float g_good_local;

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
	ros::Subscriber goal_sub = pf.subscribe("/goal", 10, goal_callback);
	motor_pub = pf.advertise<std_msgs::String>("/motor_command/path_finding", 100);
	blocked_pub = pf.advertise<std_msgs::String>("/state_change_request", 10);

	//TODO: FUNCTION ME
	std::stringstream fss;
	fss << "rover";
	std_msgs::String flush_msg;
	flush_msg.data = fss.str();
	motor_pub.publish(flush_msg);

	g_angle = 15.0f;
	g_goal_distance = 20.f;
	g_has_goal = NO_GOAL;
	g_good_local = false;

	ros::spin();
}

void print_scores(std::map<int, float>& scores) {
	for (std::map<int,float>::iterator i = scores.begin(); 
	     i != scores.end(); i++) {
		printf("%d\t%f\n", i->first, i->second);
	}
}

void goal_callback(const std_msgs::String &msg) {
	ROS_INFO("GOT CALLBACK - %s", msg.data.c_str());
	std::stringstream data(msg.data);

	if (msg.data == "stop") {
		g_has_goal = NO_GOAL;
	} else if (msg.data == "roam") {
		g_has_goal = ANY_GOAL;
	} else {
		std::string angle_str, dist_str;
		std::getline(data, angle_str, ',');
		std::getline(data, dist_str, ',');

		float angle = atof(angle_str.c_str());
		float dist = atof(dist_str.c_str());

		g_angle = angle;
		g_goal_distance = dist;
		g_has_goal = HAS_GOAL;
	}

	g_good_local = true;
}

void grid_callback(const roscv2::Grid& msg) {
	std_msgs::String blocked_msg;
	Grid grid = Grid::from_msg(msg);

	float good = msg.pct_good;

	if (good < g_good_thresh * THRESH) {
		ROS_INFO("Not enough data in grid. - %f/%f", good, g_good_thresh);
		g_good_thresh = g_good_thresh * THRESH_DECAY;
		//blocked_msg.data = -1;
	} else {
		g_good_thresh += (good - g_good_thresh) * THRESH_GROWTH;

		//print_grid(grid);

		bool direct_blocked = forward_obstacle(grid);
		bool goal_blocked = goal_obstacle(grid, g_angle, g_goal_distance);

		blocked_msg.data = direct_blocked ? "Blocked" : "Not Blocked";
		ROS_INFO("The rover is %sblocked!", direct_blocked ? "" : "not ");

		std::map<int, float> scores;
		score_directions(grid, scores);

        if (g_has_goal == ANY_GOAL) {
			move_forward(direct_blocked, scores);
        } else if (g_has_goal == HAS_GOAL) {
            if (std::abs(g_angle) <= HALF_ANGLE) {
                move_forward(direct_blocked, scores);
            } else {
                move_goal(goal_blocked, direct_blocked, g_angle, scores);
            }
        } else {
            //Nothin'
            ROS_INFO("No goal-- nothin'");
        }

        if (g_has_goal == NO_GOAL) {
        } else if (g_has_goal == ANY_GOAL || std::abs(g_angle) < HALF_ANGLE) {
			move_forward(direct_blocked, scores);
		} else {
			move_goal(goal_blocked, direct_blocked, g_angle, scores);
		} 
		//print_scores(scores);
        blocked_pub.publish(blocked_msg);
	}
}

bool forward_obstacle(const Grid& grid) {
	float square_width = grid.real_width() / (float)grid.width();
	float square_height = grid.real_height() / (float)grid.height();

	int rover_w_sq = (int)ceil((ROVER_WIDTH/2.0) / square_width);
	int max_dist_sq = (int)ceil(g_distance / square_height);

	int bound_l = (int)floor(((float)grid.width()-1.0)/2.0)-rover_w_sq;
	int bound_r = (int)ceil(((float)grid.width()-1.0)/2.0)+rover_w_sq;
	//ROS_INFO("Bounds: %d - %d", bound_l, bound_r);
	//ROS_INFO("Dist: %d", max_dist_sq);

	for (int j = 0; j < max_dist_sq; j++) {
		for (int i = bound_l; i <= bound_r; i++) {
			if (grid[i][j] != 0) return true;
		}
	}
	return false;
}

bool goal_obstacle(const Grid& grid, float angle, float dist) {
	float square_width = grid.real_width() / (float)grid.width();
	float square_height = grid.real_height() / (float)grid.height();

	int rover_w_sq = (int)ceil((ROVER_WIDTH/2.0) / square_width);
	int max_dist_sq = (int)ceil(dist / square_height);

	for (int j = 0; j < max_dist_sq; j++) {
		float angle_rad = angle * (PI / 180.f);
		int c_off = round((float)j * tan(angle_rad));
		int center = (int)floor(((float)grid.width()-1.0)/2.0) + c_off;
		for (int i = center-rover_w_sq; i <= center+rover_w_sq; i++) {
			if (0 > i || i >= grid.width()) {
				ROS_INFO("i out of bounds");
				continue;
			}
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


void move_forward(bool blocked, std::map<int, float>& scores) {
	static int turn_direction = FORWARD;

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

void move_goal(bool goal_blocked, bool direct_blocked, 
               float angle, std::map<int, float>& scores) {

	std_msgs::String move_msg;
	std::stringstream mss;

    if (!g_good_local) return;

    ROS_INFO("Angle: %f", angle);
	if (!goal_blocked) {
        g_good_local = false;
		ROS_INFO("Path to goal is clear -- rotating");	
		if (angle < 0) mss << "r340";
		else mss << "r20";

		move_msg.data = mss.str();
	} else {
		ROS_INFO("Path to goal blocked");
		if (std::abs(angle) < 45.f) { //We're facing towarsd the goal
			//Modify scores to weight one direction better?
			move_forward(direct_blocked, scores);
			return; //No more messages!
		} else {
			//Rotate towards goal so we're at least looking near it
			if (angle < 0) mss << "r340";
			else mss << "r20";
			move_msg.data = mss.str();
		}
	}
#ifdef MOVE
	motor_pub.publish(move_msg);
#endif
}
