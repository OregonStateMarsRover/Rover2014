#!/usr/bin/env python
import roslib
import rospy
from stereo_msgs.msg import DisparityImage
import struct
import numpy as np
import pdb

class obstacle_detector:
	def __init__(self):
		self.disp_callback = rospy.Subscriber("/my_stereo/disparity", DisparityImage, self.callback_disp)

	def callback_disp(self, data):
		focal = data.f
		baseline = data.T	
		disp = struct.unpack(str(data.image.width*data.image.height)+'f', data.image.data)
		disp_np = np.array(disp)
		fb = focal * baseline
		dist = fb / disp_np
		pdb.set_trace()
		

if __name__ == '__main__':
	try:
		od = obstacle_detector()
		rospy.init_node("obstacle_detector", anonymous=True)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
