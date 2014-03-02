#!/usr/bin/env python
import rospy
from std_msgs.msg import String

###################################################################
# Name: talker()
# Description: Publishes to the motor control the strings necessary
#              for creating a path
###################################################################
def talker():
   # Sets node called "motor_controller" as publishing to the
   # motor_command and on_command topics
   motor_pub = rospy.Publisher("motor_command", String)
   on_pub = rospy.Publisher("on_command", String)
   rospy.init_node("navigation")

   # Run the operation of setting the serial command string
   # The following part will publish that string every second
   while not rospy.is_shutdown():
      string = "f200b3r60"
      rospy.loginfo(string)
      motor_pub.publish(string)
      on_pub.publish(string)
      rospy.sleep(30.0)

# If an error of the node being shutdown or Ctrl+C'd occurs, it is
# excepted to avoid continuing to run
if __name__ == "__main__":
   try:
      talker()
   except rospy.ROSInterruptException:
      pass

