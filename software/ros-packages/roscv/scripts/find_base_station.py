#!/usr/bin/python2
import cv2
import roslib
roslib.load_manifest('roscv')
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs
import stereo_msgs
import std_msgs

"""
" What I need:
" Angle of view for camera(messure the width of the frame at a known height then use trig to get angle from center)
" use (angle of view/(w^2+h^2)) where the image is wxh in resolution to get the amount of angle per pixel
" Focal length of camera
" F = focal length, R = real height of square, I = Image Height, H = height in pixels of square, S = sensor height
" use (F*R*I)/(H*S) to get the distance
" turn the rover till the angle is 0, drive till we are a set distance away
"""


class FindStart():
    def __init__(self):
        self.left_image_sub = rospy.Subscriber("/my_stereo/left/image_rect_color", sensor_msgs.msg.Image, self.image)
        self.point_callback = rospy.Subscriber("/my_stereo/disparity", stereo_msgs.msg.DisparityImage, self.points)
        self.status_update = rospy.Subscriber("/find_base_station", std_msgs.msg.String, self.status)
        self.bridge = CvBridge()
        self.focal = None
        self.baseline = None
        #the hight of the camera off the ground
        self.cam_height = .5
        #the width and height of the square
        self.square = .205

        self.started = 1

    def image(self, data):
        if self.started == 0:
            return
        print "Read Image"
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.checker_board(image)
    def points(self, data):
        self.focal = data.f
        self.baseline = data.T
        #we only need the focal length from the disparity
        self.point_callback.unregister()

    def status(self, data):
        if data == "1":
            self.started = 1

    def checker_board(img):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(img, (7, 6), None)
        if ret:
            print corners
        return False

    def get_distance(self, img, height):
        if self.focal is not None:
            return (self.focal*self.square*img.shape[1])/(height*self.cam_height)
        return -1

    def get_angle(self):
        pass

if __name__ == '__main__':
    try:
        rospy.init_node("Home_search", anonymous=True)
        #handle lethal signals in order to stop the motors if the script quits
        FindStart()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass