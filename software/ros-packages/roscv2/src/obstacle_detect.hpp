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

/* Constants */
#define IMAGE_WINDOW "Image"
#define DEPTH_WINDOW "Distance"
#define OBS_WINDOW "Obstacle"

#define DOWNSCALE (1)

#define IMG_WIDTH (640/DOWNSCALE)
#define IMG_HEIGHT (480/DOWNSCALE)

#define RANGE_MIN (2.0)
#define RANGE_MAX (40.0)

#define PI (3.14159)
#define THETA (PI / 6.0)
#define MIN_H (0.10)
#define MAX_H (0.25)

#define MIN_AREA (500)

#define NUM_SLICES (20)
//#define __SLICE_DEBUG

/* Typedefs */

typedef std::vector<cv::Rect> RectList;

/* Functions */
void loop();
void get_images(sensor_msgs::Image::ConstPtr&,
                stereo_msgs::DisparityImage::ConstPtr&);
void find_obstacles(const cv::Mat&, cv::Mat&, float, float);
float get_depth_scale(float);

void init_slices(std::vector<cv::Mat>&);
void fill_slices(const cv::Mat&, std::vector<cv::Mat>&, float);
void remove_noise(cv::Mat&);
RectList calc_bboxes(cv::Mat&);

void init_cv();
void cleanup_cv();

void disparity_callback(const stereo_msgs::DisparityImage::ConstPtr&);
void image_callback(const sensor_msgs::Image::ConstPtr&);
