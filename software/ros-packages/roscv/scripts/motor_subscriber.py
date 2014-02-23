#!/usr/bin/env python
import rospy
from std_msgs.msg import String

############################################################################
# Name: callback()
# Description: Publishes a message to show the subscriber has heard from the
#              publisher
############################################################################
def callback(data):
   rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)

#########################################################################
# Name: listener()
# Description: Subcribes to the publisher for the status of the motor and
#              arms
#########################################################################
def listener():
   # Subscribes to arm_control and motor_control topics and initializes the
   # to the motor_controller node
   rospy.init_node("motor_controller", anonymous=True)
   rospy.Subscriber("motor_control", String, callback)
   rospy.Subscriber("arm_control", String, callback)
   rospy.spin()   # Keeps it running until node is closed

if __name__ == "__main__":
   listener()
