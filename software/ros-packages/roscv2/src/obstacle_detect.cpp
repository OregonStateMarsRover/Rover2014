#include "obstacle_detect.hpp"

int main(int argc, char *argv[]) {
	/* Init ROS */
	ros::init(argc, argv, "obstacle_detect");
	ros::NodeHandle n;
	ROS_INFO("obstacle_detect started");

	init_cv();

	/* Subscribe */
	/*
	ros::Subscriber disparity_sub = n.subscribe("/my_stereo/disparity",
	                                1, disparity_callback);
	ros::Subscriber image_sub = n.subscribe("/my_stereo/left/image_rect_color",
	                            1, image_callback);
	*/

	/* Spin */
	while (ros::ok()) {
		loop();
		cv::waitKey(1);
		ros::spinOnce();
	}

	cleanup_cv();

	return 0;
}

void loop() { 
	/* Grab a pair of images */
	sensor_msgs::Image::ConstPtr img_msg;
	stereo_msgs::DisparityImage::ConstPtr disp_msg;

	get_images(img_msg, disp_msg);

	/* Display raw image */
	cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(img_msg,
	                            sensor_msgs::image_encodings::BGR8);
	cv::imshow(IMAGE_WINDOW, img->image);

	/* Get disparity data */
	cv_bridge::CvImagePtr disp = cv_bridge::toCvCopy(disp_msg->image,
	                             "32FC1");
	float focal = disp_msg->f; float base = disp_msg->T;

	/* Generate depth image */
	/* Why are these so inaccurate? Calibration issue?
	float min_depth = (focal * base) / disp_msg->min_disparity;
	float max_depth = (focal * base) / disp_msg->max_disparity;
	*/
	cv::Mat full_depth = (focal * base) / disp->image;
	cv::Mat depth;
	cv::resize(full_depth, depth, cv::Size(IMG_WIDTH, IMG_HEIGHT));

	/* Display value-scaled depth image */
	cv::Mat scaled_depth = depth / RANGE_MAX;
	cv::imshow(DEPTH_WINDOW, scaled_depth);

	/* Create empty obstacle map */
	cv::Mat obstacle = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_32F);

	/* Find and display obstacles */
	find_obstacles(depth, obstacle, RANGE_MIN, 100.0);
	cv::Mat scaled_obs = obstacle / RANGE_MAX;
	cv::imshow(OBS_WINDOW, scaled_obs);
}

void find_obstacles(const cv::Mat& depth_img, cv::Mat& obstacle_img, 
                    float min, float max) {
	for (int row = depth_img.rows-1; row >= 0; row--) {
		const float *d = (const float*)depth_img.ptr(row);
		float *o = (float*)obstacle_img.ptr(row);
		for (int col = depth_img.cols-1; col >= 0; col--) {
			float depth = d[col];
			if (depth <= min /*|| depth >= max*/) continue; /* out of range */
			if (o[col] > 0) continue; /* Already an obstacle? Skip. */

			/* Valid for examination */
			float scale = get_depth_scale(depth);
			int min_row = row - (int)std::max(MIN_H * scale, 1.0);
			int max_row = row - (int)std::max(MAX_H * scale, 1.0);

			/* Make sure we don't fall off the image! */
			min_row = std::max(min_row, 0);
			max_row = std::max(max_row, 0);

			/* TODO Trade accuracy for speedup?? */
			//max_row = std::max(max_row, min_row-5);

			/* Loop over relevant image rows to search for obstacle */
			/* TODO: Optimize cache stuff? */
			bool obstacle = false;
			for(int subrow = min_row; subrow > max_row; subrow--) {
				int dx = (int)(tan(THETA) * (float)(row-subrow));
				int min_col = std::max(col - dx, 0);
				int max_col = std::min(col + dx, IMG_WIDTH);
				
				const float *sd = (const float*)depth_img.ptr(subrow);
				float *so = (float*)obstacle_img.ptr(subrow);
				for(int subcol = min_col; subcol < max_col; subcol++) {
					float subdepth = sd[subcol];
					if (subdepth <= 0) continue;
					float dz = ((float)dx) / scale; //dz is in meters
					if (depth - dz < subdepth && subdepth < depth + dz) {
						obstacle = true;
						so[subcol] = subdepth; /* TODO should it be subdepth */
					}
				}
			}
			if (obstacle) o[col] = depth;
		}
	}
}

/* TODO???? */
float get_depth_scale(float depth) {
	float focal_length = 600.0;
	float scale = focal_length / depth;
	return scale;
}

void get_images(sensor_msgs::Image::ConstPtr& im,
                stereo_msgs::DisparityImage::ConstPtr& dm) {
	im = ros::topic::waitForMessage<sensor_msgs::Image>(
	     "/my_stereo/left/image_rect_color");
	dm = ros::topic::waitForMessage<stereo_msgs::DisparityImage>(
	     "/my_stereo/disparity");
}

void init_cv() {
	ROS_INFO("Starting CV");
	cv::namedWindow(IMAGE_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(DEPTH_WINDOW, CV_WINDOW_AUTOSIZE);
	cv::namedWindow(OBS_WINDOW, CV_WINDOW_AUTOSIZE);
}

void cleanup_cv() {
	ROS_INFO("Destroying CV");
	cv::destroyWindow(IMAGE_WINDOW);
	cv::destroyWindow(DEPTH_WINDOW);
	cv::destroyWindow(OBS_WINDOW);
}
