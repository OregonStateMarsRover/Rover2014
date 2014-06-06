#!/usr/bin/env python
import roslib
import rospy
import std_msgs
import math

from std_msgs.msg import String

#TODO: THESE SUCK
ROVER_WIDTH=1.093
WHEEL_DIAMETER=0.300
WHEEL_CIRC=math.pi*WHEEL_DIAMETER
TICK_PER_ROT=540.0

class Localization:
	def __init__(self, pos=(0,0), angle=0):
		self.position = pos
		self.angle = angle
		rospy.Subscriber("encoder", String, lambda data: self.callback(data))
		self.pub = rospy.Publisher("position", String)

	def callback(self, data):
		try:
			move_type = data.data[0]
			amt = float(data.data[1:])
		except:
			print "ERROR - %s BAD DATA" % data.data
			return

		if move_type == "f":
			self.move_forward(amt)
		elif move_type == "r":
			self.rotate(amt)

		self.publish_pos()


	def move_forward(self, amt):
		x1, y1 = self.position
		rad = self.angle * math.pi / 180.0
		x2 = self.position[0] + amt * math.cos(rad)
		y2 = self.position[1] + amt * math.sin(rad)
		self.position = (x2, y2)

	def rotate(self, amt):
		self.angle = (self.angle + amt) % 360.0

	def publish_pos(self):
		x, y = self.position
		x = round(x,2)
		y = round(y,2)
		s = "%s,%s,%s" % (x, y, self.angle)
		print "Sending %s" % s
		self.pub.publish(s)

if __name__=="__main__":
	rospy.init_node("localization")
	loc = Localization()
	#TODO: SUBSCRIBE TO ENCODER TOPIC
	#TODO: PUBLISH POSITION TOPIC
	rospy.spin()
