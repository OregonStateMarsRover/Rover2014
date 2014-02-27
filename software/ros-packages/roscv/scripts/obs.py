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

DOWNSCALE=2
WIDTH=640/DOWNSCALE
HEIGHT=480/DOWNSCALE
SLICES=20
DEPTH_SCALE=1 #was 30
MIN_H = 0.10
MAX_H = 0.11
THETA = math.pi/6.0
MIN_AREA=40
DEBUG=False
MAX_RANGE=30.0

#Dumb class for timing tests
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
		self.raw_image = None #Stores the raw image from the cameras

		#Callback description
		self.disp_callback = rospy.Subscriber("/my_stereo/disparity", DisparityImage, self.callback_disp)
		self.img_callback = rospy.Subscriber("/my_stereo/left/image_rect_color", Image, self.callback_img)

		#Set up cv2 windows
		cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("obstacle", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("final", cv2.WINDOW_AUTOSIZE)

		#Set up debug cv2 windows for slices
		if DEBUG:
			for x in range(SLICES):
				cv2.namedWindow(str(x), cv2.WINDOW_AUTOSIZE)

	def callback_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		np_arr = np_arr.reshape((480,640,3))
		np_arr = cv2.resize(np_arr, (WIDTH, HEIGHT))
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

		#Downscale to the width/height we want
		dist = cv2.resize(dist, (WIDTH, HEIGHT))

		#Scale down distance to 0-1 range for display based on max range
		scale = dist / MAX_RANGE
		cv2.imshow("depth", scale)
		cv2.waitKey(1)

		#Create empty obstacle map
		obs = np.zeros((HEIGHT,WIDTH), np.uint8)

		with Timer("Find obstacles") as _:
			self.find_obstacles(dist, obs)
			""" obstacle map is now zero where there's no obsacle
			and (dist) where there's an obstacle at distance dist"""

		#Same display scaling as above
		scale_obs = obs.copy()
		scale_obs = scale_obs / MAX_RANGE
		cv2.imshow("obstacle", scale_obs)
		cv2.waitKey(1)

		#Create an array to hold the slice images
		slices = []
		for x in range(SLICES):
			#Create empty binary slice and append
			s = np.zeros((HEIGHT,WIDTH), np.uint8)
			slices.append(s)

		#Perform slicing
		with Timer("Slice") as _:
			self.fill_slices(obs, slices, MAX_RANGE)

		#Remove noise?? TODO
		with Timer("Denoise") as _:
			for x in range(len(slices)):
				slices[x] = self.remove_noise(slices[x])

		#Get bounding boxes of slices
		boxes = []
		with Timer("Bbox") as _:
			for x in slices:
				boxes.append(self.get_bboxes(x))
		
		#Show slices if debug
		if DEBUG:
			for x in range(SLICES):
				cv2.imshow(str(x), slices[x])
				cv2.waitKey(1)

		#Display pretty boxes on image
		if current_image is not None:
			ob = current_image.copy()
			ob = cv2.cvtColor(ob, cv2.COLOR_BGR2HSV)
			_max = len(boxes)

			#Draw boxes from back to front with different hues
			for l,s in enumerate(boxes[::-1]):
				hue = 120-int(float(l)/float(_max)*120)
				for x,y,w,h in s:
					cv2.rectangle(ob, (x,y), (x+w,y+h), (hue,255,255),-1)
			ob = cv2.cvtColor(ob, cv2.COLOR_HSV2BGR)
			#Mix bounding image with raw image for transparency
			current_image = cv2.addWeighted(current_image, 0.7, ob, 0.3, 0)

		#Display image if exists
		if current_image is not None:
			cv2.imshow("final", current_image)
		cv2.waitKey(1)

	#Find obstacles
	def find_obstacles(self, depth, obs):
		c = [x for x in np.ndindex(depth.shape)]
		c.reverse()
		from math import isnan, isinf
		for y, x in c:
			d = depth[y,x]
			if d < 2 or isnan(d) or isinf(d): continue #TODO
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
					if pd < 0 or isnan(pd) or isinf(pd): continue
					dz = dx / (scale * DEPTH_SCALE)
					if d - dz < pd < d + dz:
						obstacle = True
						obs[py, px] = d
			if obstacle: obs[y,x] = d
	
	#TODO: return ranges
	def fill_slices(self, obs, outs, _max):
		levels = len(outs) #Determine number of slices we're working
		_min =  obs.min() #find minimum obstacle distance
		#Maybe that should just 0?
		r = _max - _min #Range of values
		step_dist = 1.0 / float(levels)

		for y, x in np.ndindex(obs.shape): #Loop over the pixes in the obstacle map
			d = obs[y,x] #Get distance of obstacle at pixel (if present)
			if d <= 0: continue #If not present just continue
			v = d / float(r) #TODO: Fix normalization
			sl = int(v/step_dist) #Get index of slice the obstacle belongs to
			sl = min(sl, levels-1) #Make sure we don't write past the end of the array
			o = outs[sl] #Get the slice image
			o[y,x] = 255 #Set coord to 1 (obstacle)

	#Noise reduction function
	#TODO Make good
	def remove_noise(self, obs):
		obs = cv2.medianBlur(obs, 3)
		""" ????
		open_k = np.ones((3,3), np.uint8)
		obs = cv2.morphologyEx(obs, cv2.MORPH_OPEN, open_k)
		"""
		close_k = np.ones((9,9), np.uint8)
		obs = cv2.morphologyEx(obs, cv2.MORPH_CLOSE, close_k)
		return obs

	#Get bounding boxes of obstacles for a single slice
	def get_bboxes(self, obs):
		boxes = []
		#Find conotours
		contours, heir = cv2.findContours(obs.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

		for c in contours:
			#Make sure the contour is big enough area-wise
			if cv2.contourArea(c) < MIN_AREA: continue
			#Maybe check how round/what shape it is?

			#Get bounding rectangle
			x,y,w,h = cv2.boundingRect(c)

			#Store in list of boxes
			boxes.append((x,y,w,h))
		return boxes

	"""Returns scaling factor to convert world distance to pixel distance based on depth"""
	def depth_scale(self, depth):
		fl = 600.0 #TODO
		scale = fl / depth
		#print "Depth: %s\nScale: %s\n" % (depth, scale)
		return scale

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
