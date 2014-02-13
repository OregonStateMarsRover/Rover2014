#!/usr/bin/env python
import roslib
import rospy
from stereo_msgs.msg import DisparityImage
import struct
import numpy as np
import cv2
import pdb
import math

WIDTH=640/2
HEIGHT=480/2
DEPTH_SCALE=30
MIN_H = 1.0
MAX_H = 2.0
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
		dist = cv2.resize(dist, (WIDTH, HEIGHT))

		scale = dist / (dist.mean() * 3)
		cv2.imshow("depth", scale)
		obs = np.zeros((HEIGHT,WIDTH), np.uint8)

		#self.filter_dist(dist, obs, 5, 8)
		self.find_obstacles(dist, obs)
		cv2.imshow("obstacle", obs)
		#cv2.imwrite("out.png", obs)
		cv2.waitKey(1)

	def filter_dist(self, depth, obs, min_d, max_d):
		for x, y in np.ndindex(depth.shape):
			d = depth[x,y]
			if min_d < d < max_d:
				obs[x,y] = 255

	def find_obstacles(self, depth, obs):
		print "Start"
		c = [x for x in np.ndindex(depth.shape)]
		c.reverse()
		for y, x in c:
			d = depth[y,x]
			if d < 0: continue
			if obs[y,x] > 255: continue

			scale = self.depth_scale(d)
			min_row = y - int(max(MIN_H * scale, 1.0))
			max_row = y - int(max(MAX_H * scale, 1.0))
			
			min_row = max(min_row, 0)
			max_row = max(max_row, 0)	

			#px, py are the depth pixels in the 'cone' being
			#examined
			obstacle = False
			for py in range(min_row, max_row, -1):
				dx = int(math.tan(THETA) * (y-py))
				min_col = x - dx
				max_col = x + dx
				for px in range(min_col, max_col):
					pd = depth[py, px]
					if pd < 0: continue
					dz = dx / (scale * DEPTH_SCALE)
					if d - dz < pd < d + dz:
						obstacle = True
						#obs[py, px] = 255
						break
			if obstacle: obs[y,x] = 255
			else: obs[y,x] = 100
		print "Done"

	def depth_scale(self, depth):
		return 5.0
		

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
