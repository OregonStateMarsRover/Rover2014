#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__=="__main__":
	rospy.init_node("sender")
	topic = raw_input("Topic:")
	print "Publishing to topic", topic
	pub = rospy.Publisher(topic, String)
	while True:
		try:
			msg = raw_input()
		except:
			break
		pub.publish(msg)
