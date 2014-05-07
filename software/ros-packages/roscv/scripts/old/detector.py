#!/usr/bin/env python
import roslib
roslib.load_manifest('roscv')
import sys
import rospy
import cv2
from include import colorDetect
import math
import numpy as np
import struct
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError


class image_converter(object):

    def __init__(self):
        cv2.namedWindow("left", cv2.WINDOW_AUTOSIZE)

        #create our image publishers
        self.image_left_pub = rospy.Publisher("detect_left", Image)


        self.bridge = CvBridge()
        #create our subscriber
        self.left_image_sub = rospy.Subscriber("/my_stereo/left/image_rect_color", Image, self.callback_left)
        self.point_callback = rospy.Subscriber("/my_stereo/disparity", DisparityImage, self.callback_points)
        self.points = None
        self.left = None

    def callback_left(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e
        self.left = np.asarray(cv_image)
        self.transform()

    def callback_points(self, data):
        self.points = data

    def transform(self):
        if self.left == None or self.points == None:
            return
        left  = np.asarray(self.left)
        #num = pcl.pointXYZRGB(self.points)
        """ point cloud stuff
        dtype_list = [(f.name, np.float32) for f in self.points.fields]
        cloud_arr = np.fromstring(self.points.data, dtype_list)
        cloud = np.reshape(cloud_arr, (self.points.height, self.points.width))
        """

        focal = self.points.f
        baseline = self.points.T
        #next step get the disparity value d then use z = fb/d
        disp = struct.unpack(str(self.points.image.width*self.points.image.height)+'f', self.points.image.data)
        self.left = self.points = None
        detect = colorDetect.colorDetect(left, [110,30, 140], [255, 85, 255])
        boxes = detect.detect()
        for data in boxes:
            x,y,w,h = data
            cv2.rectangle(left,(x,y),(x+w,y+h),(0,255,0),10)
            minDepth = self.minDepthInRegion(disp, focal, baseline, (x, y),(x+w, y+h), left.shape[0])
            if minDepth > 0:
                cv2.putText(left, str(minDepth) + " m", (x-w/2, y+h/2), cv2.FONT_HERSHEY_COMPLEX, 1, 255)
        try:
            self.image_left_pub.publish(self.bridge.cv_to_imgmsg(cv2.cv.fromarray(left), "bgr8"))
        except CvBridgeError, e:
            print e
    def minDepthInRegion(self, cloud, focal, baseline, xrange, yrange, width):
        minn = no = 100000000000
        fb = focal*baseline
        for x in range(xrange[0], xrange[1]):
            for y in range(yrange[0], yrange[1]):
                item = cloud[y*width+x]
                if item > 0:
                    print item
                    #using formula z = fb/d
                    minn = min(minn, fb/item)
    if minn == no:
        minn = 0
        return minn


# a empty callback function
def empty(self, junk):
    pass
def main():
    ic = image_converter()
    rospy.init_node("watcher", anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main()
