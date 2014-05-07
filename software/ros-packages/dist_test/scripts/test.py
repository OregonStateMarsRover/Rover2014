#!/usr/bin/env python
import roslib
import rospy
import std_msgs

from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image

import struct
import numpy as np
import cv2
import pdb
import math

MAX_RANGE = 40.0

class Test:
	def __init__(self):
		self.raw_image = None
		cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)

	def spin(self):
		di = rospy.wait_for_message("/my_stereo/disparity", DisparityImage)
		ri = rospy.wait_for_message("/my_stereo/left/image_rect_color", Image)

		self.callback_img(ri)
		self.callback_disp(di)

	def callback_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		np_arr = np_arr.reshape((480,640,3))
		#np_arr = cv2.resize(np_arr, (WIDTH, HEIGHT))
		self.raw_image = np_arr

	def callback_disp(self, data):
		#Current image holds the image from the beginning of the detection
		current_image = None
		if self.raw_image is not None:
			current_image = self.raw_image.copy()

		#Get data from disparity/camera struct
		focal = data.f #Focal length (in pixels)
		baseline = data.T #Baseline (in meters?)

		#Read in disparity and put in a 2d array
		disp = struct.unpack(str(data.image.width*data.image.height)+'f', data.image.data)
		disp_np = np.array(disp, np.float).reshape((480,640))

		#Disparity -> Distance scaling factor
		fb = focal * baseline

		#Get distance map from disparity map + scaling factor
		dist = fb / disp_np

		#Scale down distance to 0-1 range for display based on max range
		scale = dist / MAX_RANGE
		
		real_size = (0.565, 0.838)
		sq_size = 0.229
		squares = 26

		if squares == 11:
			rect = (255,268,123,195)
		if squares == 16:
			rect = (260,295,87,130)
		if squares == 21:
			rect = (265,305,69,103)
		if squares == 26:
			rect = (273,311,55,87)

		x,y,w,h = rect
		cv2.rectangle(current_image, (x,y), (x+w,y+h), (0, 0, 255), 1)

		depth, size = self.estimate_depth(rect, dist, sq_size*squares)
		print "Depth: %.5f" % ((squares*sq_size)-depth)
		ds = (real_size[0]-size[0], real_size[1]-size[1])
		print "Size: %.5f , %.5f" % (ds[0], ds[1])

		cv2.imshow("depth", scale)
		cv2.imshow("image", current_image)
		cv2.waitKey(1)

	def estimate_depth(self, (x,y,w,h), dist, actual):
		s = 0
		i = 0
		for xp in range(x, x+w):
			for yp in range(y, y+h):
				v = dist[yp, xp]
				if v > 0:
					s += dist[yp, xp]
					i += 1
		if i == 0: i = 1
		dist = float(s)/float(i)

		width = w/self.depth_scale(actual)
		height = h/self.depth_scale(actual)
		return dist, (width, height)

	def depth_scale(self, depth):
		fl = 600.0 #TODO
		scale = fl / depth
		#print "Depth: %s\nScale: %s\n" % (depth, scale)
		return scale


if __name__=="__main__":
	try:
		test = Test()
		rospy.init_node("test", anonymous=True)
		while True:
			test.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
