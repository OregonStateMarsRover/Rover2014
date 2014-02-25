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

FX=240
FY=240
FW=40
FH=60

BX=370
BY=235
BW=40
BH=45

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

		cv2.rectangle(np_arr, (FX,FY), (FX+FW,FY+FH), (255,255,255))
		cv2.rectangle(np_arr, (BX,BY), (BX+BW,BY+BH), (255,255,255))
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
		print sum(ds[0])/len(ds[0]) 
		print sum(ds[1])/len(ds[1]) 

		sf = 20.0
		scale = dist / sf
		cv2.imshow("disp", scale)
		cv2.waitKey(1)

	def get_avg_dists(self, dist):
		dists = [[],[]]
		for y,x in np.ndindex(dist.shape):
			v = dist[y,x]
			if v <= 0: continue
			if x in range(FX,FX+FW) and y in range(FY,FY+FH):
				dists[0].append(v)
			if x in range(BX,BX+BW) and y in range(BY,BY+BH):
				dists[1].append(v)
		return dists

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()
