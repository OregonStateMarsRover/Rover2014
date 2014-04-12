#include "obstacle_detect.hpp"

ros::Publisher pub;

int main(int argc, char *argv[]) {
	/* Init ROS */
	ros::init(argc, argv, "obstacle_detect");
	ROS_INFO("obstacle_detect started");

	ros::NodeHandle n;
	pub = n.advertise<roscv2::Grid>("obstacle_grid", 10);
	init_cv();

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
	//cv::imshow(IMAGE_WINDOW, img->image);

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
	/* Not be necessary if downscale = 1 */
	cv::resize(full_depth, depth, cv::Size(IMG_WIDTH, IMG_HEIGHT));

	/* Display value-scaled depth image */
	cv::Mat scaled_depth = depth / RANGE_MAX;
	cv::imshow(DEPTH_WINDOW, scaled_depth);

	/* Create empty obstacle map */
	cv::Mat obstacle = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_32F);
	cv::Mat clear = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_32F);

	/* Find and display obstacles */
	find_obstacles(depth, obstacle, RANGE_MIN, 100.0);
	cv::Mat scaled_obs = obstacle / RANGE_MAX;
	cv::imshow(OBS_WINDOW, scaled_obs);

	/* Set up slices */
	std::vector<Slice> slices;
	init_slices(slices);
	fill_slices(obstacle, slices, RANGE_MAX);

	for (int i = 0; i < slices.size(); i++) {
		remove_noise(slices[i].mat);
	}

	/* Calculate bounding box on each slice */
	std::vector<RectList> slice_bboxes;
	for (int i = 0; i < slices.size(); i++) {
		slice_bboxes.push_back(calc_bboxes(slices[i].mat));
	}

	/* Display bounding boxes on image */
	cv::Mat boxes_image = img->image.clone();
	/* Convert box image to HSV */
	cv::cvtColor(boxes_image, boxes_image, cv::COLOR_BGR2HSV);
	/* Loop backwards-- farthest first, panter's algorithm */
	for (int i = slice_bboxes.size()-1; i >= 0; i--) {
		/* Calculate hue */
		int hue = 120 - (int)(((float)i)/((float)slice_bboxes.size())*120.0);
		cv::Scalar color = cv::Scalar(hue, 255, 255);
		for (int j = 0; j < slice_bboxes[i].size(); j++) { //TODO: Iterators???
			/* Get / resize boxes */
			cv::Rect bbox = slice_bboxes[i][j];
			bbox.x *= DOWNSCALE; bbox.y *= DOWNSCALE;
			bbox.width *= DOWNSCALE; bbox.height *= DOWNSCALE;
			/* Draw boxes */
			cv::rectangle(boxes_image, bbox, color, -1);
		}

	}
	/* Convert back to RGB */
	cv::cvtColor(boxes_image, boxes_image, cv::COLOR_HSV2BGR);

	/* Combine with image */
	cv::Mat final_image;
	cv::addWeighted(boxes_image, 0.3, img->image, 0.7, 0.0, final_image);

	/* Generate top-down image */
	cv::Mat top;
	/* Init top-down image */
	top = cv::Mat::zeros(TOP_SIZE, TOP_SIZE, CV_8UC1);
	Grid grid = Grid(GRID_WIDTH, GRID_HEIGHT, RANGE_MAX, RANGE_MAX);
	calc_topdown_grid(grid, slices, slice_bboxes, RANGE_MAX);
	publish_grid(grid);

	draw_grid(grid, top);
	calc_topdown(top, slices, slice_bboxes, RANGE_MAX);
	cv::imshow(TOP_WINDOW, top);

	cv::imshow(IMAGE_WINDOW, final_image);

#ifdef __SLICE_DEBUG
	for (int i = 0; i < NUM_SLICES; i++) {
		std::string s = "a";
		s[0] = 'a'+i;
		cv::imshow(s, slices[i]);
	}
#endif
}

void find_obstacles(const cv::Mat& depth_img, cv::Mat& obstacle_img, 
                    float min, float max) {
	ros::Time start = ros::Time::now();
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

			/* Loop over relevant image rows to search for obstacle */
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
						so[subcol] = subdepth; 
					}
				}
			}
			if (obstacle) o[col] = depth;
		}
	}
	ros::Duration duration = ros::Time::now() - start;
	ROS_INFO("find_obstacles:\t%f", duration.toSec());
}

/* TODO???? */
/* multiply to convert REAL DISTANCE to PIXEL DISTANCE */
/* divide for the opposite */
float get_depth_scale(float depth) {
	float focal_length = 600.0;
	float scale = focal_length / depth;
	return scale / (float)DOWNSCALE;
}


void init_slices(std::vector<Slice> &slices) {
	for (int i = 0; i < NUM_SLICES; i++) {
		Slice s;
		s.mat = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
		slices.push_back(s);
	}
}

void fill_slices(const cv::Mat &obs, std::vector<Slice> &slices, float max) {
	int levels = slices.size();
	float min = 0.0;
	float range = max - min;
	float step_dist = 1.0 / (float)levels;

	/* Fill min/max values for slices */
	for (int i = 0; i < levels; i++) {
		float step = step_dist*range;
		float smin = min + step*i;
		float smax = smin + step;
		slices[i].min = smin;
		slices[i].max = smax;
	}

	for (int row = 0; row < obs.rows; row++) {
		float *o = (float*)obs.ptr(row);
		for (int col = 0; col < obs.cols; col++) {
			float dist = o[col];
			if (dist <= 0) continue;
			float val = dist / range;
			int sl = (int)(val / step_dist);
			sl = std::min(sl, levels-1);

			unsigned char *o_out = slices[sl].mat.ptr(row);
			o_out[col] = 255; /* Set to max */
		}
	}
}

void remove_noise(cv::Mat &mat) {
	/* Median blur */
	cv::medianBlur(mat, mat, 3);
	/* Close kernel */
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(17,17));
	/* Morphology */
	cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, kernel);
}

RectList calc_bboxes(cv::Mat &mat ) {
	RectList boxes;
	std::vector<std::vector<cv::Point> > contours;
	/* Do edge detection?? */

	/* Find contours */
	cv::findContours(mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	/* For each contour: */
	for (int i = 0; i < contours.size(); i++) { //TODO: Iterator?
		/* Check area */
		if (cv::contourArea(contours[i]) < MIN_AREA) continue;

		/* Get bounding box */
		//TODO: Maybe poly approx first?
		cv::Rect bbox = cv::boundingRect(contours[i]);

		/* Add to list */
		boxes.push_back(bbox);
	}
	return boxes;
}

void calc_topdown(cv::Mat &top, const std::vector<Slice> &slices, 
                  const std::vector<RectList> &bboxes, float max_dist) {
	float ppm = (float)TOP_SIZE / max_dist; /* Pixels per meter */

	for (int i = 0; i < slices.size(); i++) {
		const RectList& boxes = bboxes[i];
		/* Calculate Y position for this slice */
		int max_y = TOP_SIZE - TOP_SIZE*(slices[i].min/max_dist);
		int min_y = TOP_SIZE - TOP_SIZE*(slices[i].max/max_dist);
		int y = min_y;
		int height = max_y-min_y;

		/* Get avg depth for this slice */
		float depth = (slices[i].min + slices[i].max) / 2.0;
		float scale = get_depth_scale(depth);

		cv::Scalar color = cv::Scalar(255, 255, 255);
		for (int j = 0; j < boxes.size(); j++) {
			/* Get distance (IN METERS) of bbox from center */
			int dx = boxes[j].x - (IMG_WIDTH/2);
			float dx_m = dx / scale;

			/* Get width (IN METERS) of bbox */
			float width_m = boxes[j].width / scale;

			/* Convert to pixels on top-down view */
			int x = ppm * dx_m + (TOP_SIZE/2);
			int width = ppm * width_m;

			/* Draw */
			cv::Rect r = cv::Rect(x, y, width, height);
			cv::rectangle(top, r, color, -1);
		}
	}
}

//TODO: mostly copied from above
void calc_topdown_grid(Grid &grid, const std::vector<Slice> &slices, 
                       const std::vector<RectList> &bboxes, float max_dist) {
	for (int i = 0; i < slices.size(); i++) {
		const RectList& boxes = bboxes[i];

		/* Get avg depth and scale for this slice */
		float depth = (slices[i].min + slices[i].max) / 2.0;
		float scale = get_depth_scale(depth);

		int y = (int)((depth/max_dist)*grid.height());

		if (y < 0 || y >= grid.height()) continue;

		cv::Scalar color = cv::Scalar(255, 255, 255);
		for (int j = 0; j < boxes.size(); j++) {
			/* Get distance (IN METERS) of bbox from center */
			int dx = boxes[j].x - (IMG_WIDTH/2);
			float dx_m = (dx / scale)+(max_dist/2);

			/* Get width (IN METERS) of bbox */
			float width_m = boxes[j].width / scale;

			int min_x = (dx_m/max_dist)*grid.width();
			int max_x = ((dx_m+width_m)/max_dist)*grid.width();

			for (int i = min_x; i <= max_x; i++ ) {
				if (i < 0 || i >= grid.width()) continue;
				grid[i][y] = 1;
			}
		}
	}
}

void publish_grid(const Grid &grid) {
		roscv2::Grid msg;
		grid.populate_msg(msg);
		pub.publish(msg);
}

void draw_grid(const Grid& grid, cv::Mat& top) {
	cv::Scalar color = cv::Scalar(127,0,0);
	float step_x = TOP_SIZE/grid.width();
	float step_y = TOP_SIZE/grid.height();

	for (int y = 0; y < grid.height(); y++) {
		for (int x = 0; x < grid.height(); x++) {
			int _y = grid.height() - (y+1);
			if (grid[x][y] == 0) continue;
			int py = (int)(_y * step_y);
			int px = (int)(x * step_x);

			cv::Rect r = cv::Rect(px, py, step_x, step_y);
			cv::rectangle(top, r, color, -1);
		}
	}
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
	cv::namedWindow(TOP_WINDOW, CV_WINDOW_AUTOSIZE);
#ifdef __SLICE_DEBUG
	for (int i = 0; i < NUM_SLICES; i++) {
		std::string s = "a";
		s[0] = 'a'+i;
		cv::namedWindow(s, CV_WINDOW_AUTOSIZE);
	}
#endif
}

void cleanup_cv() {
	ROS_INFO("Destroying CV");
	cv::destroyWindow(IMAGE_WINDOW);
	cv::destroyWindow(DEPTH_WINDOW);
	cv::destroyWindow(OBS_WINDOW);
	cv::destroyWindow(TOP_WINDOW);
#ifdef __SLICE_DEBUG
	for (int i = 0; i < NUM_SLICES; i++) {
		std::string s = "a";
		s[0] = 'a'+i;
		cv::destroyWindow(s);
	}
#endif
}
