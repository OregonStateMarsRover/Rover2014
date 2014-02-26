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

DOWNSCALE=1
WIDTH=640/DOWNSCALE
HEIGHT=480/DOWNSCALE

FX=230#+10
FY=225#+10
FW=60#-10*2
FH=92#-10*2

BX=360#+10
BY=228#+10
BW=60#-10*2
BH=65#-10*2

class obstacle_detector:
	def __init__(self):
		self.disp_callback = rospy.Subscriber("/my_stereo/disparity", DisparityImage, self.callback_disp)
		self.img_callback = rospy.Subscriber("/my_stereo/left/image_rect_color", Image, self.callback_img)
		cv2.namedWindow("disp", cv2.WINDOW_AUTOSIZE)
		cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)

	def callback_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		np_arr = np_arr.reshape((480,640,3))
		np_arr = cv2.resize(np_arr, (WIDTH, HEIGHT))

		self.draw_box(np_arr)
		cv2.imshow("img", np_arr)
		cv2.waitKey(1)


	def callback_disp(self, data):
		focal = data.f
		baseline = data.T	
		disp = struct.unpack(str(data.image.width*data.image.height)+'f', data.image.data)
		disp_np = np.array(disp, np.float).reshape((480,640))
		fb = focal * baseline
		dist = fb / disp_np
		dist = cv2.resize(dist, (WIDTH, HEIGHT))

		ds = self.get_avg_dists(dist)
		nd = self.print_dist("Near", ds[0])
		nd = 4
		self.print_length("Near", nd, (FW, FH))
		fd = self.print_dist("Far", ds[1])
		fd = 6
		self.print_length("Far", fd, (BW, BH))
		print ""

		sf = 20.0
		scale = dist / sf
		self.draw_box(scale)
		cv2.imshow("disp", scale)
		cv2.waitKey(1)

	def draw_box(self, img):
		cv2.rectangle(img, (FX,FY), (FX+FW,FY+FH), (255,255,255))
		cv2.rectangle(img, (BX,BY), (BX+BW,BY+BH), (255,255,255))

	def print_dist(self, s, l):
		d = sum(l) / len(l)
		print s,"\t",d
		return d

	def print_length(self, s, dist, l):
		print s,"\t",
		for x in l:
			print self.calc_length(dist, x),
		print ""

	def get_avg_dists(self, dist):
		from math import isnan, isinf
		dists = [[],[]]
		for y,x in np.ndindex(dist.shape):
			v = dist[y,x]
			if v <= 0 or isnan(v) or isinf(v): continue
			if x in range(FX,FX+FW) and y in range(FY,FY+FH):
				dists[0].append(v)
			if x in range(BX,BX+BW) and y in range(BY,BY+BH):
				dists[1].append(v)
		return dists

	#63x56
	#64x42
	def calc_length(self, dist, pl):
		fl = 600.0 #TODO
		length = dist * float(pl) / fl
		return length

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
