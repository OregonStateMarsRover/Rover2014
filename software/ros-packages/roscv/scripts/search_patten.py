#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading
import math
import time

INITIAL_DISTANCE = 20*10
#SO DUMB
def command_gen(dist=20):
    yield "f"+str(int(INITIAL_DISTANCE))
    while True:
        yield "f"+str(int(dist))
        yield "r90"
        yield "f"+str(int(dist))
        yield "r90"
        dist += 20

class search_pattern(object):
    def __init__(self):
        rospy.Subscriber("/state", String, self.state_callback)
        rospy.Subscriber("/motor_status", String, self.motor_callback)
        self.pub = rospy.Publisher("/motor_command/search_pattern", String)
        self.motor_status = None
        self.state_status = None
        self.motor_ready = True
        self.searchSpiralRect()

    def stop(self):
        self.running = False

    def state_callback(self, data):
        self.state_status = data.data
        print "State is:", self.state_status

    def motor_callback(self, data):
        self.motor_status = data.data
        if self.motor_status == "free":
            self.motor_ready = True
        print "Motor is:", self.motor_status, self.motor_ready

    def can_run(self):
        return self.motor_ready and self.state_status == "SearchPattern"

    def searchSpiralRect(self):
        gen = command_gen()
        while True:
            if self.can_run():
                command = gen.next()
                self.pub.publish(command)
                print "Publishing",command
                self.motor_ready = False
            time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("search_pattern")
    thread = threading.Thread(target=search_pattern)
    thread.daemon = True
    thread.start()
    rospy.spin()
