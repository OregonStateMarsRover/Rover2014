#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading
import math

class search_pattern(object):

    def __init__(self):
	self.motor = rospy.Publisher("/motor_command/path_finding/")

    def blocked(self):
	while rospy.wait_for_message("/state", String).data != "SearchPattern":
	    pass

    def searchSpiralRect(self, start, end):
	if i == 0:
	    self.blocked()
	    self.motor.publish("f10r90f10r90")
	    start += 1

	for n in range(start, end):
	    distance = 2+floor(start/4.0)
	    self.blocked()
	    self.motor.publish("f%dr90" % int(distance))

    def returnHome(self):
	pos = rospy.wait_for_message("/position", String).data
	pos = pos.split(",")
	distance = math.sqrt(float(pos[0])**2 + float(pos[1])**2)
	self.blocked()
	self.motor.publish("r%df%d" % (math.atan2(float(pos[0]), float(pos[1])) , int(distance*10))
