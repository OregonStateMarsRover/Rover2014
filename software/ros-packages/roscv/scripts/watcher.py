#!/usr/bin/env python
import roslib
roslib.load_manifest('roscv')
import sys
import rospy
import cv2
import pcl
from math import floor
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import calibrate

class image_converter(object):
	
	def __init__(self):
		cv2.namedWindow("left", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("right", cv2.WINDOW_AUTOSIZE)
		#create our image publishers
		self.image_left_pub = rospy.Publisher("detect_left", Image)
		self.image_right_pub = rospy.Publisher("detect_right", Image)
		
		#a place to put the last images published by a node
		self.left_prev = None
		self.right_prev = None
		
		self.bridge = CvBridge()
		#create our subscriber
		self.left_image_sub = rospy.Subscriber("/my_stereo/left/image_raw", Image, self.callback_left)
		self.right_image_sub = rospy.Subscriber("/my_stereo/right/image_raw", Image, self.callback_right)
		
		self.cal = calibrate.calibrate(["cal/left.txt", "cal/right.txt"])
		if not self.cal.load():
			self.cal.calibrate()
		
		
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
		window_size = 9
		min_disp = 0
		num_disp = 128
		unique = 10
		speckleWindow = 0
		speckleRange = 9
		stereo = cv2.StereoSGBM(minDisparity = min_disp,
		    numDisparities = num_disp,
		    SADWindowSize = window_size,
		    preFilterCap = 4,
		    uniquenessRatio = unique,
		    speckleWindowSize = speckleWindow,
		    speckleRange = speckleRange,
		    disp12MaxDiff = 2,
		    P1 = 600,
		    P2 = 2400,
		    fullDP = False
	    )
		disparity = stereo.compute(leftMono, rightMono)
		norm = disparity.copy()
		cv2.normalize(disparity, norm, 0, 255, cv2.cv.CV_MINMAX, cv2.cv.CV_8U)
		cv2.imshow("left", leftMono)
		cv2.imshow("right", disparity)	
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
