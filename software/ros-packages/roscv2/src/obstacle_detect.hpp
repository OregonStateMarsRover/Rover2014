#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#define IMAGE_WINDOW "Image"
#define DEPTH_WINDOW "Distance"

#define RANGE_MIN (0)
#define RANGE_MAX (40)

void init_cv();
void cleanup_cv();

void disparity_callback(const stereo_msgs::DisparityImage::ConstPtr& msg);
void image_callback(const sensor_msgs::Image::ConstPtr& msg);
