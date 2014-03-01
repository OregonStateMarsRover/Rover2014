#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "obstacle_detect");
	ros::NodeHandle n;
	
	ROS_INFO("obstacle_detect started");

	cv::namedWindow("TestWindow", CV_WINDOW_AUTOSIZE);

	while (ros::ok()) {
		cv::waitKey(1);
		ros::spinOnce();
	}

	cv::destroyWindow("TestWindow");
	return 0;
}
