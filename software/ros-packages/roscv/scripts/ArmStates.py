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
        self.movable = True
        self.item_count = 0
        self.arm = rospy.Publisher("arm_commands", std_msgs.msg.String)
        self.status = rospy.Publisher("/arm_state", std_msgs.msg.String)
        self.arm_state = rospy.Subscriber("/arm_state_change", std_msgs.msg.String, self.get_object)
        self.thread = threading.Thread(target=self.status)
        #self.move(self.cal())
        self.change_state("docked")

    def change_state(self, state):
        try:
            nxt = self.states[state]()
        except KeyError:
            return
        if self.movable:
            self.movable = False
            print "switching from", self.state, "to", state
            prev = self.states[self.state]()
            self.move(prev["from"])
            self.move(nxt["to"])
            self.state = state
            self.movable = True
    def move(self, commands):
        self.movable = False
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
        position = [(8, 90, 350, 240),
                    (0, 90, 350, 240),
                    (0, 90, 260, 250),
                    (0, 90, 220, 200)]
        back = [(8, 90, 350, 300),
                (0, 0, 350, 300)]
        return {"to": position, "from": back}

    def move_home(self):
        position = [(0, 0, 350, 300)]
        back = [(0, 0, 350, 300)]
        return {"to": position, "from": back}

    def grab(self):
        position = [(0, 0, 40, 300), (2, 0, 0, 220)]
        back = [(2, 0, 350, 300)]
        return {"to": position, "from": back}

    def store(self):
        place = self.item_count
        #position will be a list lists where each list is the instructions to get to the next pocket
        position = [[(10, 0, 350, 300), (0, 180, 285, 280)],
                    [(10, 0, 350, 300), (0, 154, 240, 340)]]
        #return will lift then rotate to home
        back = [(8, 0, 350, 300)]
        try:
            return {"to": position[place], "from": back}
        except IndexError:
            return {"to": [(0, 0, 350, 300)], "from": [(0, 0, 350, 300)]}
    def cal(self):
        return [(8, 0, 350, 300)]

    def get_object(self, data):
        if data.data == "docked":
            self.change_state("docked")
        else:
            print "got state"
            self.change_state("home")
            self.change_state("grab")
            self.change_state("store")
            self.change_state("home")
            self.item_count += 1
            self.change_state("docked")

    def status(self):
        while True:
            if self.state == "docked":
                self.status.publish("docked")
            else:
                self.status.publish("pickup")
            time.sleep(.25)

if __name__ == '__main__':
    try:
        rospy.init_node("ArmStateMachine", anonymous=True)
        #handle lethal signals in order to stop the motors if the script quits
        start = ArmState()
        #start.points(rospy.wait_for_message("/my_stereo/disparity", stereo_msgs.msg.DisparityImage))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
