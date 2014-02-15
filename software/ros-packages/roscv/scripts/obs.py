#!/usr/bin/env python
import roslib
import rospy
from stereo_msgs.msg import DisparityImage
import struct
import numpy as np
import cv2
import pdb
import math

DOWNSCALE=3
WIDTH=640/DOWNSCALE
HEIGHT=480/DOWNSCALE
SLICES=5
DEPTH_SCALE=30
MIN_H = 1.5
MAX_H = 2.0
THETA = math.pi/4.0

class Timer:
	def __init__(self, name):
		import time
		self.name = name

	def __enter__(self):
		import time
		self.starttime = time.time()

	def __exit__(self, type, value, traceback):
		import time
		self.endtime = time.time()
		print self.name, ":", self.endtime - self.starttime

class obstacle_detector:
	def __init__(self):
		self.disp_callback = rospy.Subscriber("/my_stereo/disparity", DisparityImage, self.callback_disp)
		cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("obstacle", cv2.WINDOW_AUTOSIZE)
		for x in range(SLICES):
			cv2.namedWindow(str(x), cv2.WINDOW_AUTOSIZE)

	def callback_disp(self, data):
		focal = data.f
		baseline = data.T	
		disp = struct.unpack(str(data.image.width*data.image.height)+'f', data.image.data)
		disp_np = np.array(disp, np.float).reshape((480,640))
		fb = focal * baseline
		dist = fb / disp_np
		dist = cv2.resize(dist, (WIDTH, HEIGHT))

		sf = dist.mean() * 3.0
		scale = dist / sf
		cv2.imshow("depth", scale)
		cv2.waitKey(1)
		obs = np.zeros((HEIGHT,WIDTH), np.uint8)

		#self.filter_dist(dist, obs, 5, 8)
		with Timer("Find obstacles") as _:
			self.find_obstacles(dist, obs)

		scale_obs = obs.copy()
		scale_obs = scale_obs / sf
		cv2.imshow("obstacle", scale_obs)
		cv2.waitKey(1)

		
		slices = []
		for x in range(SLICES):
			s = np.zeros((HEIGHT,WIDTH), np.uint8)
			slices.append(s)

		with Timer("Slice") as _:
			self.filter(obs, slices, sf)

		with Timer("Denoise") as _:
			for x in range(len(slices)):
				slices[x] = self.remove_noise(slices[x])
		
		for x in range(SLICES):
			cv2.imshow(str(x), slices[x])
			cv2.waitKey(1)


	def find_obstacles(self, depth, obs):
		c = [x for x in np.ndindex(depth.shape)]
		c.reverse()
		for y, x in c:
			d = depth[y,x]
			if d < 0: continue
			if obs[y,x] > 0: continue

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
				min_col = max(0, min_col)
				max_col = min(WIDTH-1, max_col)
				for px in range(min_col, max_col):
					pd = depth[py, px]
					if pd < 0: continue
					dz = dx / (scale * DEPTH_SCALE)
					if d - dz < pd < d + dz:
						obstacle = True
						obs[py, px] = d
			if obstacle: obs[y,x] = d
	
	#TODO: return ranges
	def filter(self, obs, outs, _max):
		levels = len(outs)
		_min =  obs.min()
		#_max = obs.max()+1
		#_max = obs.mean()*6
		r = _max - _min
		steps = 1.0 / float(levels)

		for y, x in np.ndindex(obs.shape):
			d = obs[y,x]
			if d <= 0: continue
			v = d / float(r)
			sl = int(v/steps)
			sl = min(sl, levels-1)
			o = outs[sl]
			o[y,x] = 255

	def remove_noise(self, obs):
		obs = cv2.medianBlur(obs, 3)
		open_k = np.ones((3,3), np.uint8)
		close_k = np.ones((8,8), np.uint8)
		#obs = cv2.morphologyEx(obs, cv2.MORPH_OPEN, open_k)
		obs = cv2.morphologyEx(obs, cv2.MORPH_CLOSE, close_k)
		return obs
			

	def depth_scale(self, depth):
		return 10.0/DOWNSCALE
		

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
