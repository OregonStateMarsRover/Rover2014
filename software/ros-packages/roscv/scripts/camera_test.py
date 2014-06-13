#!/usr/bin/env python
import roslib
import rospy
import std_msgs

from sensor_msgs.msg import Image

import numpy as np
import cv2

class stereo_test:
	def __init__(self):
		"""
		cv2.namedWindow("left", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("right", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("sub", cv2.WINDOW_AUTOSIZE)
		"""

	def spin(self):
		li = rospy.wait_for_message("/my_stereo/left/image_rect_color", Image)
		ri = rospy.wait_for_message("/my_stereo/right/image_rect_color", Image)
		left = self.callback_img(li, "left")
		right = self.callback_img(ri, "right")
		sub = self.sub_img(left, right)
		return np.sum(sub)

	def callback_img(self, data, window):
		np_arr = np.fromstring(data.data, np.uint8)
		np_arr = np_arr.reshape((480,640,3))
		np_arr = cv2.cvtColor(np_arr, cv2.COLOR_BGR2LAB)
		#cv2.imshow(window, np_arr)
		#cv2.waitKey(1)
		return np_arr

	def sub_img(self, data1, data2):
		sub = abs(data1-data2)
		#cv2.imshow("sub", sub)
		return sub

if __name__ == "__main__":
	rospy.init_node("camera_test", anonymous=True)
	st = stereo_test()
	count = 0
	s = 0
	while count < 25:
		count += 1
		s += st.spin()
	print (s/count)/(1000000.0)
