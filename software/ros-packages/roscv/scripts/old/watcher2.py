#!/usr/bin/env python
import roslib
roslib.load_manifest('roscv')
import sys
import rospy
import cv2
from math import floor
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter(object):
	
	def __init__(self):
		cv2.namedWindow("left", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("right", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("settings", cv2.WINDOW_AUTOSIZE)
		#create our image publishers
		self.image_left_pub = rospy.Publisher("detect_left", Image)
		self.image_right_pub = rospy.Publisher("detect_right", Image)
		
		#a place to put the last images published by a node
		self.left_prev = None
		self.right_prev = None
		
		self.bridge = CvBridge()
		#create our subscriber
		self.left_image_sub = rospy.Subscriber("/my_stereo/left/image_rect_color", Image, self.callback_left)
		self.right_image_sub = rospy.Subscriber("/my_stereo/right/image_rect_color", Image, self.callback_right)
		
		#create trackbars for editing settings
		cv2.createTrackbar("window_size", "settings", 10, 255, self.empty)
		cv2.createTrackbar("min_disp", "settings", 1, 255, self.empty)
		cv2.createTrackbar("num_disp", "settings", 128, 3200, self.empty)
		cv2.createTrackbar("unique", "settings", 5, 1000, self.empty)
		cv2.createTrackbar("speckleWindow", "settings", 0, 1000, self.empty)
		cv2.createTrackbar("speckleRange", "settings", 9, 10000, self.empty)
		
	def callback_left(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e
		self.left_prev = cv_image
		self.transform()	
		
	def callback_right(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e
		self.right_prev = cv_image
		self.transform()

	def transform(self):
		if self.left_prev == None or self.right_prev == None:
			return
		left  = np.asarray(self.left_prev)
		right = np.asarray(self.right_prev)
		self.left_prev = self.right_prev = None
		
		leftMono = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
		rightMono = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
		
		# disparity range is tuned for 'aloe' image pair
		window_size = cv2.getTrackbarPos("window_size", "settings")\
		min_disp = cv2.getTrackbarPos("min_disp", "settings")
		num_disp = int(16*floor(cv2.getTrackbarPos("num_disp", "settings")/16))
		unique = cv2.getTrackbarPos("unique", "settings")
		speckleWindow = cv2.getTrackbarPos("speckleWindow", "settings")
		speckleRange = cv2.getTrackbarPos("speckleRange", "settings")
		stereo = cv2.StereoSGBM(minDisparity = min_disp,
		    numDisparities = num_disp,
		    SADWindowSize = window_size,
		    uniquenessRatio = unique,
		    speckleWindowSize = speckleWindow,
		    speckleRange = speckleRange,
		    disp12MaxDiff = 2,
		    P1 = 8*3*window_size**2,
		    P2 = 32*3*window_size**2,
		    fullDP = False
	    )
		disparity = stereo.compute(leftMono, rightMono)
		norm = disparity.copy()
		cv2.normalize(disparity, norm, 10, 255, cv2.cv.CV_MINMAX, cv2.cv.CV_8U)
		cv2.imshow("left", leftMono)
		cv2.imshow("right", norm)
		cv2.waitKey(1)
		""" Publisher dont know what to do with it yet
		try:
			self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError, e:
			print e
		"""
		# a empty callback function
	def empty(self, junk):
		pass
def main():
	ic = image_converter()
	rospy.init_node("watcher", anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	main()
