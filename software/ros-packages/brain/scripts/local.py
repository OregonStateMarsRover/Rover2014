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
		self.last_left = None
		self.last_right = None
		rospy.Subscriber("encoder", String, lambda data: self.callback(data))
		self.pub = rospy.Publisher("position", String)

	def callback(self, data):
		left, right = map(float,data.data.split(','))
		print "Got %s / %s" % (left, right)
		if self.last_left is not None and self.last_right is not None:
			dl = left - self.last_left
			dr = right - self.last_right
			if dl > 0 and dr > 0:
				dl *= 2
				dr *= 2
			self.move(dl, dr)
		self.last_left = left
		self.last_right = right

	#Assuming small angles
	def move(self, left_tick, right_tick):
		#http://www.ridgesoft.com/articles/trackingposition/TrackingPosition.pdf
		left_rot = left_tick / TICK_PER_ROT
		right_rot = right_tick / TICK_PER_ROT

		left_dist = left_rot * WHEEL_CIRC
		right_dist = right_rot * WHEEL_CIRC

		dist = 0.5 * (left_dist + right_dist)
		rot = (left_dist - right_dist) / (ROVER_WIDTH * 2.0 * math.pi)
		#TODO: CHECK ABOVE

		xp = self.position[0] + dist * math.cos(self.angle)
		yp = self.position[1] + dist * math.sin(self.angle)
		self.position = (xp,yp)
		ap = self.angle + rot
		while ap > 2.0 * math.pi:
			ap -= 2.0 * math.pi
		while ap < 0:
			ap += 2.0 * math.pi
		self.angle = ap
		self.publish_pos()

	def publish_pos(self):
		x, y = self.position
		deg = self.angle * (180.0 / math.pi)
		s = "%s,%s,%s" % (x, y, deg)
		print "Sending %s" % s
		self.pub.publish(s)

if __name__=="__main__":
	rospy.init_node("localization")
	loc = Localization()
	#TODO: SUBSCRIBE TO ENCODER TOPIC
	#TODO: PUBLISH POSITION TOPIC
	rospy.spin()
