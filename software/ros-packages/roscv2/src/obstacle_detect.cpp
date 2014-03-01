#include "obstacle_detect.hpp"

int main(int argc, char *argv[]) {
	/* Init ROS */
	ros::init(argc, argv, "obstacle_detect");
	ros::NodeHandle n;
	ROS_INFO("obstacle_detect started");

	init_cv();

	/* Subscribe */
	ros::Subscriber disparity_sub = n.subscribe("/my_stereo/disparity",
	                                1, disparity_callback);
	ros::Subscriber image_sub = n.subscribe("/my_stereo/left/image_rect_color",
	                            1, image_callback);

	/* Spin */
	while (ros::ok()) {
		cv::waitKey(1);
		ros::spinOnce();
	}

	cleanup_cv();

	return 0;
}

void disparity_callback(const stereo_msgs::DisparityImage::ConstPtr& msg) {
	//ROS_INFO("Disparity callback!");
	cv_bridge::CvImagePtr cv_disp = cv_bridge::toCvCopy(msg->image,
	                                "32FC1");
	float focal = msg->f; float baseline = msg->T;
	cv::Mat depth = (focal * baseline) / cv_disp->image; //Get depth map

	cv::Mat scaled_depth = depth / RANGE_MAX; //Scale it for display
	cv::imshow(DEPTH_WINDOW, scaled_depth);
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg) {
	//ROS_INFO("Image callback!");
	cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg,
	                            sensor_msgs::image_encodings::BGR8);
	cv::imshow(IMAGE_WINDOW, img->image);
}

void init_cv() {
	ROS_INFO("Starting CV");
	cv::namedWindow(IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DEPTH_WINDOW, CV_WINDOW_AUTOSIZE);
}

void cleanup_cv() {
	ROS_INFO("Destroying CV");
	cv::destroyWindow(IMAGE_WINDOW);
	cv::destroyWindow(DEPTH_WINDOW);
}
