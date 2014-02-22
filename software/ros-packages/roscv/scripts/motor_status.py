#!/usr/bin/env python
import rospy
from std_msgs.msg import String

################################################################################
# Name: talker
# Description: Publishes to the pathing that the motor or the arm is either busy
#		or not
################################################################################
def talker():
   # Sets node called "motor_busy" as publishing to the motor_control topic
   # The "motor_busy" :will also publish similar info to the arm_control topic
   motor_pub = rospy.Publisher("motor_control", String)
   arm_pub = rospy.Publisher("arm_control", String)
   rospy.init_node("motor_busy")

   # Run the operation of checking if the motor/arm is busy or not and publishing
   # that information. It sleeps between publishes for a second
   while not rospy.is_shutdown():
      busy = 0
      rospy.loginfo(busy)
      motor_pub.publish(str(busy))
      arm_pub.publish(str(busy))
      rospy.sleep(1.0)

# The main running of the publisher
# If an error of the node being shutdown or Ctrl+C'd is excepted to avoid
# continuing to run
if __name__ == "__main__":
   try:
      talker()
   except rospy.ROSInterruptException:
      pass
