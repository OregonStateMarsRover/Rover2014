#include <iostream>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <roscv2/Grid.h>

/* Constants */
#define IMAGE_WINDOW "Image"
#define DEPTH_WINDOW "Distance"
#define OBS_WINDOW "Obstacle"
#define TOP_WINDOW "Top Down"

#define DOWNSCALE (1)

#define IMG_WIDTH (640/DOWNSCALE)
#define IMG_HEIGHT (480/DOWNSCALE)

#define TOP_SIZE (400)
#define GRID_WIDTH (40)
#define GRID_HEIGHT (40)

#define RANGE_MIN (2.0)
#define RANGE_MAX (40.0)

#define PI (3.14159)
#define THETA (PI / 6.0)
#define MIN_H (0.10)
#define MAX_H (0.25)

#define MIN_AREA (500/(DOWNSCALE*DOWNSCALE))

#define NUM_SLICES (40)
//#define __SLICE_DEBUG

/* Typedefs */

typedef std::vector<cv::Rect> RectList;

struct Slice {
	float min, max;
	cv::Mat mat;
};

class Grid {
private:
	int **ptr;
	int _width, _height;
	float _real_width, _real_height;

	void init() {
		for (int i = 0; i < _width; i++) {
			for (int j = 0; j < _height; j++) {
				ptr[i][j] = 0;
			}
		}
	}
public:
	Grid(int w, int h, float w_m, float h_m) {
		_width = w; _height = h;
		_real_width = w_m; _real_height = h_m;
		ptr = new int*[_width];
		for (int i = 0; i < _width; i++) ptr[i] = new int[_height];
		init();
	}

	~Grid() {
		for (int i = 0; i < _width; i++) delete[] ptr[i];
		delete ptr;
	}

	int width() const { return _width; }
	int height() const { return _height; }

	int const * operator[](int i) const {
		return (int const *)ptr[i];
	}
	int * operator[](int i) {
		return ptr[i];
	}
	void populate_msg(roscv2::Grid &msg) {
		msg.width = _width;
		msg.height = _height;
		msg.width_m = _real_width;
		msg.height_m = _real_width;
		for (int j = 0; j < _height; j++) {
			for (int i = 0; i < _width; i++) {
				msg.data.push_back(ptr[i][j]);
			}
		}
	}
};

/* Functions */
void loop();
void get_images(sensor_msgs::Image::ConstPtr&,
                stereo_msgs::DisparityImage::ConstPtr&);
void find_obstacles(const cv::Mat&, cv::Mat&, float, float);
float get_depth_scale(float);

void init_slices(std::vector<Slice>&);
void fill_slices(const cv::Mat&, std::vector<Slice>&, float);
void remove_noise(cv::Mat&);
RectList calc_bboxes(cv::Mat&);
void calc_topdown(cv::Mat&, const std::vector<Slice>&, const std::vector<RectList>&, float);
void calc_topdown_grid(Grid&, const std::vector<Slice>&, const std::vector<RectList>&, float);
void draw_grid(const Grid&, cv::Mat&);

void init_cv();
void cleanup_cv();

void disparity_callback(const stereo_msgs::DisparityImage::ConstPtr&);
void image_callback(const sensor_msgs::Image::ConstPtr&);
