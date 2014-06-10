#!/usr/bin/env python
import roslib
import rospy
import std_msgs

from std_msgs.msg import String

if __name__=="__main__":
    rospy.init_node("echoer")
    pub = rospy.Publisher("motor_command", String)
    while True:
        s = raw_input()
        pub.publish(s)
