#!/usr/bin/env python
import roslib
import rospy
import std_msgs
import math

#TODO: THESE SUCK
ROVER_WIDTH=1.093
WHEEL_DIAMETER=0.284
WHEEL_CIRC=math.pi*WHEEL_DIAMETER

class Localization:
	def __init__(self, pos=(0,0), angle=0):
		self.position = pos
		self.angle = angle

	#Two candidate functions for movement

	#left/right_rot is the number of rotations in the left/right wheels
	def move_arc(self, left_rot, right_rot):
		#Set up enum
		RIGHT,LEFT = range(2)

		left_dist = left_rot * WHEEL_CIRC
		right_dist = right_rot * WHEEL_CIRC
		
		#r1 is outer arc, r2 is inner arc
		r1 = max(left_dist, right_dist)
		r2 = min(left_dist, right_dist)

		#If the left wheel is turning *more*, we're turning right
		if r1 == left_dist:
			direction == RIGHT
		else:
			direction == LEFT

		theta = (r1 - r2) / ROVER_WIDTH
		if theta != 0:
			x = r1 / theta
		else:
			x = float("inf")
		#TODO: Finish?

	#Assuming small angles
	def move(self, left_rot, right_rot):
		#http://www.ridgesoft.com/articles/trackingposition/TrackingPosition.pdf
		RIGHT,LEFT=range(2)

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


if __name__=="__main__":
	rospy.init_node("localization")
	loc = Localization()
	#TODO: SUBSCRIBE TO ENCODER TOPIC
	#TODO: PUBLISH POSITION TOPIC
	rospy.spin()
