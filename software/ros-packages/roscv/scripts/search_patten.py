#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading
import math
import time

def points_gen(init_pos, init_angle, dist):
    x,y = init_pos
    angle = init_angle
    turns = 0

    while True:
        rad = angle * math.pi / 180.0
        x = round(x + dist * math.cos(rad), 2)
        y = round(y + dist * math.sin(rad), 2)
        angle = (angle + 90) % 360
        turns += 1
        if turns == 2:
            dist += 2.0 
            turns = 0
        yield (x, y)

def close_to(pos, point):
    x, y = pos
    px, py = point

    dx = px - x
    dy = py - y
    dist = math.sqrt(dx * dx + dy * dy)

    #return dist < 5.0
    return True

def angle_to(pos, point):
    x, y = pos
    px, py = point
    dx = px - x
    dy = py - y

    rad = math.atan2(dx, dy)
    angle = rad * 180.0 / math.pi
    angle = angle % 360
    return round(angle, 2)

def dist_to(pos, point):
    x, y = pos
    px, py = point
    dx = px - x
    dy = py - y
    dist = math.sqrt(dx * dx + dy * dy)
    print dx, dy, dist
    return round(math.sqrt(dist), 2)

class search_pattern(object):
    def __init__(self):
        self.at_point = False
        self.pub = rospy.Publisher("goal", String)
        self.searchSpiralRect(2)

    def blocked(self):
        while rospy.wait_for_message("/state", String).data != "SearchPattern":
            pass

    def searchSpiralRect(self, start):
        self.current_pos = (0,0)
        self.current_angle = 0
        pg = points_gen(self.current_pos, self.current_angle, start)
        next_point = None
        
        while True:
            #wait for proximity
            if next_point is None or close_to(self.current_pos, next_point):
                next_point = pg.next()

            angle = angle_to(self.current_pos, next_point)
            goal_dist = dist_to(self.current_pos, next_point)
            s = "%s,%s" % (angle, goal_dist)
            self.pub.publish(s)
            time.sleep(1)

    def update_pos(self, data):
        x, y, angle = map(float, data.data.split(','))
        self.current_pos = (x, y)
        self.current_angle = angle
        print "GOT: ", self.current_pos, self.current_angle

"""
    def returnHome(self):
        pos = rospy.wait_for_message("/position", String).data
        pos = pos.split(",")
        distance = math.sqrt(float(pos[0])**2 + float(pos[1])**2)
        self.blocked()
        self.motor.publish("r%df%d" % (math.atan2(float(pos[0]), float(pos[1])) , int(distance*10))
"""

if __name__ == "__main__":
    rospy.init_node("search_pattern")
    sp = search_pattern()
    rospy.Subscriber("position", String, lambda data: sp.update_pos(data))
