#!/usr/bin/env python
import sys
import rospy

from subprocess import call
from std_msgs.msg import String

class Camera:
    def __init__(self):
        self.stereo_on = False
        self.arm_on = False
        self.arm_timer = 0

        rospy.Subscriber("/armCam/image_raw", String, self.arm_callback)
        rospy.Subscriber("/my_stereo/left/image_rect_color", String, self.stereo_callback)

        self.proc_pub = rospy.Publisher("process_mgmt", String)

        rospy.Timer(rospy.Duration(1), self.reset_arm_cam)
        rospy.Timer(rospy.Duration(1) , self.reset_stereo)

    def arm_callback(self, data):
        self.arm_timer = 0

    def stereo_callback(self, data):
        self.stereo_timer = 0

    def reset_arm_cam(self, event=None):
        if self.arm_timer > 5:
            print "Resetting USB and arm camera"
            self.reset_usb()
            self.proc_pub.publish("restart arm_camera")
            self.arm_timer = 0
        else:
            self.arm_timer = self.arm_timer + 1

    def reset_stereo(self, event=None):
        if self.arm_timer > 5:
            print "Resetting USB and stereo cameras"
            self.reset_usb()
            self.proc_pub.publish("restart stereo")
            self.arm_timer = 0
        else:
            self.arm_timer = self.arm_timer + 1

    def reset_usb(self):
        #call(["bash", "/
        print "in reset_usb"




if __name__=='__main__':
    rospy.init_node('restart_camera')
    cam = Camera()
    rospy.spin()
