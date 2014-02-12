#!/usr/bin/env python
import roslib
import rospy
from stereo_msgs.msg import DisparityImage
import struct
import numpy as np
import cv2
import pdb
import math

MIN_H = 0.1
MAX_H = 1.0
THETA = math.pi/4.0

class obstacle_detector:
	def __init__(self):
		self.disp_callback = rospy.Subscriber("/my_stereo/disparity", DisparityImage, self.callback_disp)
		cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("obstacle", cv2.WINDOW_AUTOSIZE)

	def callback_disp(self, data):
		focal = data.f
		baseline = data.T	
		disp = struct.unpack(str(data.image.width*data.image.height)+'f', data.image.data)
		disp_np = np.array(disp, np.float).reshape((480,640))
		fb = focal * baseline
		dist = fb / disp_np
		#pdb.set_trace()

		scale = dist / (dist.mean() * 3)
		cv2.imshow("depth", scale)
		obs = np.zeros((480,640))

		self.find_obstacles(dist, obs)
		cv2.imshow("obstacle", obs)
		cv2.waitKey(1)

	def find_obstacles(self, depth, obs):
		for x, y in np.ndindex(depth.shape):
			d = depth[x,y]
			if d < 0: continue
			if obs[x,y] == 255: continue


			if 5 < depth[x,y] < 10:
				obs[x,y] = 255

	def depth_scale(self, depth):
		return 30.0
		

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
