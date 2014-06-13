#!/usr/bin/python2
import cv2
import roslib
roslib.load_manifest('roscv')
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sensor_msgs
import stereo_msgs
import stereo_msgs.msg
import std_msgs
import math
import threading
import signal
import sys

""" 1525 1750 = 60 degrees/ 640 = degrees per pixel = .09375
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
        #self.point_callback = rospy.Subscriber("/my_stereo/disparity", stereo_msgs.msg.DisparityImage, self.points)
        self.status_update = rospy.Subscriber("/find_base_station", std_msgs.msg.String, self.status)
        self.motor = rospy.Publisher("/motor_command/find_base_station", std_msgs.msg.String)
        self.change_state = rospy.Publisher("/state_change_request", std_msgs.msg.String)
        self.bridge = CvBridge()
        self.focal = None
        self.baseline = None
        #angle per pixel
        self.app = .09375
        #the height of the camera off the ground
        self.cam_height = .25
        #the width and height of the square
        self.square_back = .095
        self.square = 0.1778
        self.dim = (5, 3)
        self.dim_small = (4, 3)
        self.started = 1
        self.image_id = 0
        self.image_read = 0
        self.searching = False
        self.search_turn = False
        self.data = None
        self.image_thread = threading.Thread(target=self.start_thread)
        self.image_thread.daemon = True
        self.image_thread.start()


    def start_thread(self):
        self.left_image_sub = rospy.Subscriber("/my_stereo/left/image_rect_color", sensor_msgs.msg.Image, self.set_data)
        self.state_sub = rospy.Subscriber("/state", std_msgs.msg.String, self.set_state)
        rospy.spin()

    def set_data(self, data):
        #print "got image"
        self.data = data

    def set_state(self, data):
        #print "'","SearchPattern" in data.data,"'"
        #FindBaseStation FindBaseStationFinal
        if "BaseStation" in data.data or "SearchPattern" in data.data:
            self.started = 1
        else:
            self.started = 0

    def image(self):
        if self.started == 0:
            return
        if self.data == None:
            return
        print "starting checkerboard"
        image = self.bridge.imgmsg_to_cv2(self.data, "bgr8")
        self.checker_board(image)
        self.data = None

    def points(self, data):
        self.focal = data.f/6000
        self.baseline = data.T

    def status(self, data):
        if data == "1":
            self.started = 1

    def checker_board(self, img):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(img, self.dim, None)
        ret_back, corners_back = cv2.findChessboardCorners(img, self.dim_small, None)
        print "Checkerboard:",
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if ret:
            print "forward"
            #cv2.drawChessboardCorners(gray, self.dim, corners, ret)
            height = self.get_height(img, corners)
            center = self.get_center(corners)
            distance = self.get_distance(gray, height)
            angle = self.get_angle(center)
            print height, center, distance, angle

        if ret_back:
            print "back"
            height = -self.get_height(img, corners_back, "small")
            center = self.get_center(corners_back)
            distance = self.get_distance(gray, height, "back")
            angle = self.get_angle(center)
            print height, center, distance, angle

        if ret or ret_back:
            self.change_state.publish("Base Station Found")
            self.motor.publish("flush")
            self.motor.publish("rover")
	
	if ret:
	    threshold = math.asin(.5/distance)
            distance -= 10
            if distance > 20:
                distance = int(distance*(3.0/4))
            else:
                self.change_state.publish("Base Station Found Final")
            if -threshold < angle < threshold:
                print "Moving forward", distance, "angle was", angle
                self.motor.publish("f%d" % (int(distance)))
            else:
                if angle < 0:
                    angle = 360+angle
                print "Rotating", angle, "and moving forward", distance
                self.motor.publish("r%df%d" % (int(angle), int(distance)))
            print distance, angle, threshold
            if distance < 20 and (angle < max(threshold, 8) or angle > min(360-threshold, 352)):
                print "Final form entered"
                self.searching = False
                self.search_turn = False
                self.change_state.publish("Found Base Station Final")
            else:
                self.searching = True
                self.searh_turn = False
            while rospy.wait_for_message("/motor_status", std_msgs.msg.String).data == "busy":
                pass
	
	if ret_back:
	    distance *= 2
	    self.motor.publish("r45f%dr45%d" % (int(distance), int(distance))) 
            while rospy.wait_for_message("/motor_status", std_msgs.msg.String).data == "busy":
                pass

        elif self.searching:
            if not self.search_turn:
                self.motor.publish("r315")
                self.search_turn = True
            else:
                self.motor.publish("r10")
        return False

    def get_height(self, img,  grid, size="large"):
        if size == "large":
            dim = self.dim
        else:
            dim = self.dim_small
            
        start = grid[0][0][1]
        end = grid[-dim[0]][0][1]
        cv2.line(img, tuple(grid[0][0]), tuple(grid[-dim[0]][0]), (255, 0, 0))
        return end-start

    def get_distance(self, img, height, dim="large"):
        self.focal = (0.5 * img.shape[1] / math.tan(0.5 * 65 * math.pi / 180))*(4.2/1000.0);
        if self.focal is not None:
            if dim == "large":
                return (2.0 / 2.7) *(self.focal*(self.square*(self.dim[0]-2))*img.shape[1])/(height*self.cam_height)
            else:
                return (self.focal*(self.square_back*(self.dim_small[0]-2))*img.shape[1])/(height*self.cam_height)	
        return -1

    def get_skew(self, img, grid):
        delta_left = (grid[self.dim[0]][0][1]-grid[0][0][1])
        left_distance = (self.focal*self.square*img.shape[0])/(delta_left*self.cam_height)
        delta_right = (grid[(self.dim[0]-1)+self.dim[0]][0][1]-grid[self.dim[0]-1][0][1])
        right_distance = (self.focal*self.square*img.shape[0])/(delta_right*self.cam_height)
        delta_lr = right_distance-((left_distance**2)+(right_distance**2)-((self.square*7)**2))/(2*right_distance)
        left_angle = math.radians(self.get_angle(grid[self.dim[0]][0][0]))
        right_angle = math.radians(self.get_angle(grid[(self.dim[0]-1)+self.dim[0]][0][0]))
        b = self.square*6
        rh = right_distance/math.cos(right_angle)
        lh = left_distance/math.cos(left_angle)
        return 3*math.degrees(math.asin((rh-lh)/b))
        """if left_distance > right_distance:
            if right_angle > 0:
                #case 2
                theta3 = math.radians(90)-right_angle
                theta4 = math.acos(((left_distance**2)-(right_distance**2)-(b**2))/(-2*right_distance*b))
                return math.degrees(theta4-theta3)
            else:
                #case 6
                theta3 = math.radians(90)+right_angle
                theta4 = math.acos(((left_distance**2)-(right_distance**2)-(b**2))/(-2*right_distance*b))
                return math.degrees(theta4-theta3)
        else:
            if left_angle > 0:
                #case 5
                theta3 = math.radians(90)+left_angle
                theta4 = math.acos(((right_distance**2)-(left_distance**2)-(b**2))/(-2*left_distance*b))
                return math.degrees(theta4-theta3)
            else:
                #case 1
                theta3 = math.radians(90)-left_angle
                theta4 = math.acos(((right_distance**2)-(left_distance**2)-(b**2))/(-2*left_distance*b))
                return math.degrees(theta4-theta3)

        return math.degrees(math.asin(delta_lr/(self.square*7)))"""

    def get_center(self, grid, side="front"):
        if side == "front":
            return grid[-self.dim[0]/2][0][0]
        else:
            return grid[-self.dim_back[0]/2][0][0]

    def get_angle(self, center):
        return (center*self.app)-30

#TODO: Any cleanup??

if __name__ == '__main__':
    try:
        global interrupt
        interrupt = False
        def handler(sig, f):
            global interrupt
            interrupt = True
            sys.exit()
        rospy.init_node("Home_search", anonymous=True)
        #handle lethal signals in order to stop the motors if the script quits
        signal.signal(signal.SIGINT, handler)
        start = FindStart()
        #start.points(rospy.wait_for_message("/my_stereo/disparity", stereo_msgs.msg.DisparityImage))
        while not interrupt:
            #start.image(rospy.wait_for_message("/my_stereo/left/image_rect_color", sensor_msgs.msg.Image))
            start.image()
    except rospy.ROSInterruptException:
        pass
