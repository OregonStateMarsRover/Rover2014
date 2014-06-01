#!/usr/bin/python2
__author__ = 'brenemal'

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
import time
import threading

class ArmState(object):
    def __init__(self):
        self.states = {"docked": self.docked, "home": self.move_home, "grab": self.grab, "store": self.store}
        self.state = "home"
        self.item_count = 0
        self.arm = rospy.Publisher("arm_commands", std_msgs.msg.String)
        self.arm_state = rospy.Subscriber("/arm_state", std_msgs.msg.String, self.get_object)

    def change_state(self, state):
        try:
            nxt = self.states[state]()
        except KeyError:
            return
        print "switching from", self.state, "to", state
        prev = self.states[self.state]()
        self.move(prev["from"])
        self.move(nxt["to"])
        self.state = state

    def move(self, commands):
        for l in commands:
            self.wait_for_arm()
            print "running", l
            self.arm.publish("%d, %d, %d, %d" % l)
            time.sleep(1)

    def wait_for_arm(self):
        print "starting wait"
        while rospy.wait_for_message("/arm_controller", std_msgs.msg.String).data == "Not":
            pass
        print "ending wait"

    def docked(self):
        position = [(0, 94, 320, 250),
                    (0, 94, 260, 300),
                    (0, 94, 220, 220)]
        back = [(0, 94, 260, 300),
                (0, 94, 320, 250),
                (8, 0, 350, 300)]
        return {"to": position, "from": back}

    def move_home(self):
        position = [(0, 0, 300, 350)]
        back = [(0, 0, 300, 350)]
        return {"to": position, "from": back}

    def grab(self):
        position = [(0, 0, 0, 300), (2, 0, 0, 240)]
        back = [(2, 0, 350, 300)]
        return {"to": position, "from": back}

    def store(self):
        place = self.item_count
        #position will be a list lists where each list is the instructions to get to the next pocket
        position = [[(10, 0, 350, 300), (0, 183, 275, 330)],
                    [(10, 0, 350, 300), (0, 154, 240, 340)]]
        #return will lift then rotate to home
        back = [(8, 0, 350, 300)]
        try:
            return {"to": position[place], "from": back}
        except KeyError:
            return {"to": [(0, 0, 350, 300)], "from": [(0, 0, 350, 300)]}

    def get_object(self, data):
        print "got state"
        self.change_state("home")
        self.change_state("grab")
        self.change_state("store")
        self.item_count += 1
        self.change_state("home")
        self.change_state("docked")


if __name__ == '__main__':
    try:
        rospy.init_node("ArmStateMachine", anonymous=True)
        #handle lethal signals in order to stop the motors if the script quits
        start = ArmState()
        #start.points(rospy.wait_for_message("/my_stereo/disparity", stereo_msgs.msg.DisparityImage))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
