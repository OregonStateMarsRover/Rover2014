#!/usr/bin/env python
import sys
import numpy as np
import cv2
import time
import random
import math
try:
    import roslib
    roslib.load_manifest('roscv')
    from cv_bridge import CvBridge, CvBridgeError
    import rospy
    import std_msgs
    import sensor_msgs
    USE_ROS = True
except ImportError:
    USE_ROS = False

GUI = True
"""
" See the comments in the color filter code since it is exactly the same code
"""
def get_threshold_image(frame, lower, upper):
    #convert frame to HSV to make it easier to work with
    frame_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #use your custom function to filter by color range
    #default :[110,50,50] [130,255,255]
    lower = np.array(lower)
    upper = np.array(upper)
    image_thresh = cv2.inRange(frame_image, lower, upper)
    #bitwise and adds the color back
    #image_thresh = cv2.bitwise_and(frame_image, frame_image, mask=image_thresh)
    return image_thresh


def contour_detect(thresh_image):
    #make a copy so that the contour get doesnt mess stuff up
    count_image = thresh_image.copy()
    #get the contours from the treshold image copy
    contours, hierarchy = cv2.findContours(count_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #foreach contour found draw a box around the area
    area = 0
    rect = 0
    for cnt in contours:
        #get rid of really small boxes
        narea = cv2.contourArea(cnt)
        print narea
        if area < narea < 5000:
            rect = cv2.boundingRect(cnt)
            area = narea
    print rect
    return rect


def shape_matches(templates, image, cap, smallest):
    #make a copy so that the contour get doesnt mess stuff up
    count_image = image.copy()
    #get the contours from the treshold image copy
    contours, hierarchy = cv2.findContours(count_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #foreach contour found draw a box around the area
    area = 0
    rect = 0
    match = 2
    count = 0
    for cnt in contours:
        #get rid of really small boxes
        narea = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        if cap > narea > smallest:
            for template in templates:
                tcontour, th = cv2.findContours(template.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                mm = .2
                temp = tcontour[0]
                for cnt2 in tcontour:
                    tm = cv2.matchShapes(cnt2, cnt, 3, 0.0)
                    if tm < mm:
                        mm = tm
                        temp = cnt2
                if mm < match and mm != 0:
                    match = mm
                    rect = (x, y, w, h, mm)
    if match != 2:
        print match
    if match > .2:
        return 0
    else:
        return rect




def thick_cluster(image):
    """
    " create empty image with an 8 pixel buffer on both ends
    " for i pixels in row:
    "    for n pixels column:
    "        if n is white:
    "            add factors to other pixels in row
    "            add factor to other pixels in column
    """
    shape = list(image.shape)
    shape[0] += 5
    shape[1] += 5
    cluster = np.zeros(shape, image.dtype)
    factors = [0, 44, 32, 26, 18, 5]
    for y in range(0, shape[0] - 8):
        for x in range(0, shape[1] - 8):
            if image[y][x] > 150:
                for i in range(1, 6):
                    cluster[y][x + i] += factors[i]
                    cluster[y + i][x] += factors[i]
    return cluster


def get_skel(img):
    _, img = cv2.threshold(img, 170, 255, cv2.THRESH_BINARY)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    skel = np.zeros(img.shape, img.dtype)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    done = False
    while not done:
        temp = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        temp = cv2.bitwise_or(temp, temp)
        temp = cv2.bitwise_and(img, temp, temp)
        skel = cv2.bitwise_or(skel, temp, skel)
        img = cv2.erode(img, element)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(img)
        done = (maxVal == 0)
    return skel


def match_object(img, hook_skel, puck_skel):
    lower = np.array([0, 0, 200], dtype=np.uint8)
    upper = np.array([255, 255, 255], dtype=np.uint8)
    skel = cv2.inRange(img.copy(), lower, upper)
    skel_height = skel.shape[0]/4
    skel = skel[skel_height:, :]
    #skel = get_skel(img)
    #temp_clustering = thick_cluster(temp_skel)
    #clustering = thick_cluster(skel)
    #_, skel = cv2.threshold(skel, 10, 255, cv2.THRESH_BINARY)
    rect = shape_matches(puck_skel, skel, 1000, 100)
    rect2 = shape_matches(hook_skel, skel, 1000, 100)
    if rect != 0 and rect2 != 0:
        rect = rect if rect[4] < rect2[4] else rect2
    elif rect == 0:
        rect = rect2
    if rect != 0:
        rect = list(rect)
        rect[1] += skel_height
        rect = tuple(rect)
    return rect, skel


def draw_bounding_box(original, rect):
    x, y, w, h, m = rect
    #cv2.imshow("test_kmean", clustering)
    #x, y, w, h = contour_detect(clustering)
    cv2.rectangle(original, (x, y), (x + w, y + h), 255, 5)
    return m


def test_matching(hook_skel, puck_skel, lower_bound, upper_bound, test_count):
    right = 0
    m_right = 0.0
    wrong = 0
    skip_count = 0
    m_wrong = 0.0
    random.seed(time.time())
    for num in range(test_count):
        img = cv2.imread("test_images/images/image%03d.png" % random.randint(lower_bound, upper_bound))
        try:
            original = img.copy()
        except AttributeError:
            continue
        rect, skel = match_object(img, hook_skel, puck_skel)
        if rect == 0:
            print "none found"
            skip_count += 1
            continue
        m = draw_bounding_box(original, rect)
        cv2.imshow("skel", skel)
        cv2.imshow("original", original)
        key = cv2.waitKey(0) & 0xFF
        if key == 97:
            right += 1
            m_right += m
        elif key == 115:
            wrong += 1
            m_wrong += m
        elif key == 113:
            break
    total = right + wrong + skip_count
    print "You got ", float(right) / float(total), "percent correct", \
        float(wrong)/float(total), "percent wrong and", float(skip_count)/float(total), " Not Found"
    print "when correct your average match value is:", m_right / right
    print "when wrong your average match value is:", m_wrong / (wrong+.001)


def skel_templates():
    img = cv2.imread('test_images/image002.png')
    base = "/home/rover/ros/rover_ws/src/roscv/scripts/"
    template = [cv2.imread(base + 'test_images/template/hockey/template1.png'),
                cv2.imread(base + 'test_images/template/hockey/template2.png'),
                cv2.imread(base + 'test_images/template/hockey/template3.png'),
                cv2.imread(base + 'test_images/template/hook/template1.png'),
                cv2.imread(base + 'test_images/template/hook/template2.png'),
                cv2.imread(base + 'test_images/template/hook/template3.png'),
                cv2.imread(base + 'test_images/template/hook/template4.png'),
                cv2.imread(base + 'test_images/template/hook/template5.png'),
                cv2.imread(base + 'test_images/template/hook/template6.png'),
                cv2.imread(base + 'test_images/template/hook/template7.png')]
    temp_skel = []
    for x, temp in enumerate(template):
        temp_skel.append(get_skel(temp))
    return temp_skel


def main():
    temp_skel = skel_templates()
    #1-124: hockey puck, 125-742 is hook
    test_matching(temp_skel[3:], temp_skel[:3], 125, 742, 100)
    cv2.destroyAllWindows()


class RosDetect():

    def __init__(self):
        self.forward = False
        self.bridge = CvBridge()
        self.state = "searching"
        self.template_skeletons = skel_templates()
        self.state_change = rospy.Publisher("/state_change_request", std_msgs.msg.String)
        self.motor = rospy.Publisher("/motor_command/object_detection", std_msgs.msg.String)
        self.arm = rospy.Publisher("/arm_state_change", std_msgs.msg.String)
        self.left_image_sub = rospy.Subscriber("/armCam/image_raw", sensor_msgs.msg.Image, self.get_image)
        self.objects = rospy.Publisher("objects/position", std_msgs.msg.String)
        rospy.init_node("object_recognition", anonymous=True)

    def convert_arm(self, x, y):
        return x, y

    def get_image(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        print self.state
        if self.state == "searching":
            self.detect_objects(image)
        elif self.state == "pickup":
            try:
                rect = self.detect_objects(image)
                self.track_to_pickup(image, rect)
            except TypeError:
                pass

    def detect_objects(self, image):
        try:
            image = image.copy()
            rect, skel = match_object(image, self.template_skeletons[3:], self.template_skeletons[:3])
            if GUI:
                cv2.imshow("skel", skel)
            if rect == 0:
                #print "none found"
                if GUI:
                    cv2.waitKey(1)
                return
            draw_bounding_box(image, rect)
            if GUI:
                cv2.imshow("original", image[image.shape[0]/4:, :])
                cv2.imshow("original2", image)
                cv2.waitKey(1)
            if self.state == "searching":
                print "switching states"
                self.state_change.publish("Object Found")
                self.state = "pickup"
            return rect
        except CvBridgeError, e:
            print e

    def track_to_pickup(self, image, rect):
        #line it up on forward axis which will require a function
        
        self.motor.publish("controller") 
        if rect[0] < 176:
            self.motor.publish("left15right25")
            time.sleep(.1)
        elif rect[0] > 206:
            self.motor.publish("left25right15")
            time.sleep(.1)
        elif 20 < image.shape[0] - rect[1] > 60:
            if not self.forward:
                self.motor.publish("left25right25")
                time.sleep(.01)
                self.forward = True
            self.motor.publish("left22right22")
            time.sleep(.1)
        else:
            print "picking up"
            self.forward = False
            self.motor.publish("rover")
            self.state_change.publish("At Object")
            self.arm.publish("pickup")
            while rospy.wait_for_message("/arm_status", std_msgs.msg.String).data == "pickup":
                pass
            self.state_change.publish("Object Retrieved")
            self.state = "searching"
        self.motor.publish("left20right20") 
        self.motor.publish("rover")
        self.state_change.publish("Object Retrieved")

if __name__ == "__main__":
    if USE_ROS:
        print "starting ros"
        detect = RosDetect()
        rospy.spin()
    else:
        main()
