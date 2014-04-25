#include <iostream>
#include <algorithm>
#include <cmath>

#include <omp.h>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "Grid.hpp"

/* Options */
#define CV_OUTPUT
//#define __SLICE_DEBUG

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

#define NUM_SLICES (10)

/* Typedefs */

typedef std::vector<cv::Rect> RectList;

struct Slice {
	float min, max;
	cv::Mat mat;
};


/* Functions */
void loop();
void get_images(sensor_msgs::Image::ConstPtr&,
                stereo_msgs::DisparityImage::ConstPtr&);
void find_obstacles(const cv::Mat&, cv::Mat&, float, float);
float get_depth_scale(float);

void img_callback(const sensor_msgs::Image::ConstPtr&);
void disp_callback(const stereo_msgs::DisparityImage::ConstPtr&);

void init_slices(std::vector<Slice>&);
void fill_slices(const cv::Mat&, std::vector<Slice>&, float);
void remove_noise(cv::Mat&);
RectList calc_bboxes(cv::Mat&);
void calc_topdown(cv::Mat&, const std::vector<Slice>&, const std::vector<RectList>&, float);
void publish_grid(const Grid &grid);
void calc_topdown_grid(Grid&, const std::vector<Slice>&, const std::vector<RectList>&, float);
void draw_grid(const Grid&, cv::Mat&);

void init_cv();
void cleanup_cv();

void disparity_callback(const stereo_msgs::DisparityImage::ConstPtr&);
void image_callback(const sensor_msgs::Image::ConstPtr&);
