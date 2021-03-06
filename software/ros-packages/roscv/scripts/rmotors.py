#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
import itertools #Used to break apart string
import collections #used for the command queue
import threading

import signal
import sys

import math

import os
import serial
from serial.tools import list_ports


def handler(signum, frame):
    print "Lethal signal received sending motor kill signal and exiting"
    if con != None:
        con.m.change(0, 0, 1)
        con.q.clear()
        con.m.maintain()
    sys.exit(1)

class SerialHandler(object):
    baud = 9600
    timeout = 3
    port = ""
    ser = None

    def __init__(self, baud=9600, timeout=3):
        self.baud = baud
        self.timeout = timeout

    def list_serial_ports(self):
        for port in list_ports.comports():
            if "ttyUSB" in port[0]:
                yield port[0]

    def get_control_port(self, check_string):
        for port in list(self.list_serial_ports()):
            try:
                ser = serial.Serial(port, self.baud, timeout=self.timeout)
                line_check = []
                for i in range(0, 2):
                    line_check.append(ser.readline())
                    time.sleep(2)
                if any(check_string in string for string in line_check):
                    print "Port90-+ Found:", port
                    self.port = port
                    self.ser = ser
                    break
            except serial.SerialException:
                pass

    def write(self, char):
        self.ser.write(char)

    def read(self, count):
        return self.ser.read(count)


class Motor(object):

    serial = None
    left = 127
    right = 127
    estop = 0
    right_speed = 127
    left_speed = 127


    def __init__(self):
        self.ramp_rate = 1
        self.stopped = False
        while True:
            try:
                self.serial = SerialHandler()
                self.serial.get_control_port("ID: MainDrive")
                time.sleep(2)
                self.serial.write('r')
                time.sleep(2)
                open(os.devnull, 'w')
                break
            except:
                pass

    def maintain(self, event=None):
        self.send_packet()

    def change(self, l, r, e):
        self.left = l
        self.right = r
        self.estop = e

    def send_packet(self):

        if self.left == None or self.right == None:
                return
        if self.right_speed != self.right:
            delta = self.right - self.right_speed
            if abs(delta) > self.ramp_rate:
                    self.ramp_rate += .5
                    self.right_speed += int(self.ramp_rate if delta > 0 else -self.ramp_rate)
                    
            else:
                    self.right_speed = self.right
                    self.ramp_rate = 1
        
        if self.left_speed != self.left:
            delta = self.left - self.left_speed
            if abs(delta) > self.ramp_rate:
                    self.ramp_rate += .5
                    self.left_speed += int(self.ramp_rate if delta > 0 else -self.ramp_rate)
                    #print "left", self.left_speed, "right", self.right_speed
            else:
                    self.left_speed = self.left
                    self.ramp_rate = 1

        if self.left_speed != 127 or self.right_speed != 127:
            print "L", self.left, self.left_speed
            print "R", self.right, self.right_speed
        self.serial.write(chr(255))
        self.serial.write(chr(self.estop))
        self.serial.write(chr(self.left_speed))
        self.serial.write(chr(self.right_speed))
        self.serial.write(chr(0 ^ self.left_speed ^ self.right_speed))
        self.serial.write(chr(255))
        while not self.read_packet:
            pass
        time.sleep(.02)

    def read_packet(self):
        read = self.serial.read(3)
        if ord(read[0]) == 255 and ord(read[2]) == 255:
            if (ord(read[1]) & 1) == 1:
                self.stopped = False
                self.estop = 0
            elif ord(read[1]) == 0:
                self.stopped = True
                self.estop = 1
            if (ord(read[1]) & 11) > 10:
                time.sleep(5)
        return False


class RosController(object):

    q = None
    pub = None
    m = None
    mode = "rover"
    speed = 0.75

    def __init__(self, status_to, commands_from):
        self.q = collections.deque()
        self.pub = rospy.Publisher(status_to, String)
        self.thread = None
        self.distance = 0
        self.m = Motor()
        self.left_ticks = 0
        self.right_ticks = 0
        self.encoder = rospy.Publisher("/encoder", String)
        rospy.Subscriber(commands_from, String, self.read_commands)

        rospy.Timer(rospy.Duration(.001), self.m.maintain)
        rospy.Timer(rospy.Duration(.05), self.publish_status)

    def publish_status(self, event=None):
        free = len(self.q) == 0 and self.thread is None
        if free:
            msg = "free"
        else:
            msg = "busy"
        self.pub.publish(msg)

    def read_commands(self, commands):
        length = len(self.q)
        commands = str(commands).replace("data:", "").replace(" ", "")
        command_list = ["".join(x) for _, x in itertools.groupby(commands, key=str.isdigit)]
        #flush the queues
        if self.mode == "rover":
            self.m.change(self.meters_to_char(0),self.meters_to_char(0),0)
        if self.mode == "rover":
            if command_list[0] in "fbsr":
                self.q.extend(command_list)
            else:
                #invalid commands, we will need to handle later
                pass
        else:
            o_left = self.m.left
            o_right = self.m.right
            for i in range(0, len(command_list), 2):
                if command_list[i] == "left":
                    amt = float(command_list[i+1])-20.0
                    o_left = self.meters_to_char(amt/10.0)
                elif command_list[i] == "right":
                    amt = float(command_list[i+1])-20.0
                    o_right = self.meters_to_char(amt/10.0)
                elif command_list[i] == "rover":
                    self.mode = "rover"
            #print "C", o_left, o_right
            self.m.change(o_left, o_right, 0)

        if command_list[0] == "rover":
            self.mode = "rover"
        elif command_list[0] == "controller":
            self.mode = "controller"

        if self.mode == "rover":
            if length == 0 and command_list[0] != "flush":
                self.update()
            elif command_list[0] == "flush":
                self.q.clear()
                self.unset_thread()

    def meters_to_char(self, speed):
        #speed must be a positive number
        speed += 2.0
        #get the normalized ratio
        speed = 255*(speed/4.0)
        #ensure its an int where 0 <= speed <= 255
        return max(min(255, int(speed)), 0)

    def unset_thread(self):
        if self.thread is not None:
            self.thread.cancel()
            self.thread = None


class MotorController(RosController):

    def update(self, event=None):
        if len(self.q) == 0:
            if self.mode == "rover":
                self.m.change(self.meters_to_char(0), self.meters_to_char(0), self.m.estop)
            return
        action = self.q[0]
        value = self.q[1]
        
        #pop to elements off the deque
        self.q.popleft()
        self.q.popleft()

        if action == "f" or action == "b":
            spd = self.meters_to_char(self.speed if action == "f" else -self.speed)
            self.m.change(spd, spd, 0) 
            val_int = int(value)
            val_int = val_int if action == "f" else -val_int
            value = str(val_int)
            self.wait_distance(value)

        elif action == "s":
            #ensure the speed is between -2.0 and 2.0 m/s(which is the assumed max speed)
            self.speed = max(min(int(value), 20), -20)/10.0
            print self.speed

        elif action == "r":
            value = int(value)
            if value < 180:
                self.m.change(self.meters_to_char(.75), self.meters_to_char(-.75), 0)
            else:
                self.m.change(self.meters_to_char(-.75), self.meters_to_char(.75), 0)
            self.wait_angle(value)

    def pub_ticks(self, command, distance):
        self.encoder.publish(str(command)+str(distance))

    def wait_distance(self, distance):
        start = time.time()
        #the ratio between the actual and theoretical meters per second
        a_mps = .3*44.0
        if self.speed == 0:
            length = 0
        else:
            length = abs(float(distance))/(self.speed*a_mps)
        self.distance = length
        self.thread = MotorStopperTimer(self.update, self.unset_thread, self.distance, distance, self.pub_ticks, "f")
        self.thread.start()


    def wait_angle(self, angle):
        start = time.time()
        angle = float(angle)
        if angle > 180:
            angle = 360 - angle
        #assume one degree a second
        dps = 1.0/44.0
        self.distance = angle*dps

        if self.thread is not None:
            self.thread.cancel()
        self.thread = MotorStopperTimer(self.update, self.unset_thread, self.distance, angle, self.pub_ticks, "r")
        self.thread.start()


class MotorStopperTimer(threading.Thread):
    def __init__(self, update, unset, duration, distance, pub_ticks, movement="f"):
        print "TIMER"
        threading.Thread.__init__(self)
        self.update = update
        self.unset = unset
        self.last_time = time.time()
        self.time = time.time()+duration
        self.event = threading.Event()
        self.done = False
        self.distance = distance
        self.type = movement
        self.pub_ticks = pub_ticks
        self.mps = (float(distance)/10.0)/duration
        self.aps = float(distance)/duration

    def run(self):
        while not self.event.is_set():
            #print "Running! %s %s" % (time.time(), self.time)
            if time.time() > self.time or self.done:
                self.finish_pub()
                break
            self.event.wait(.05)
        now = time.time()
        dt = now - self.last_time
        if self.type == "f":
            meter_distance = self.mps * dt
            if meter_distance >= 0.1:
                self.pub_ticks(self.type, round(meter_distance,2))
                self.last_time = now
        elif self.type == "r":
            angle = self.aps * dt
            if angle > 5:
                self.pub_ticks(self.type, round(angle,1))
                self.last_time = now
        self.unset()
        self.update()

    def cancel(self):
        self.done = True

    def finish_pub(self):
        now = time.time()
        dt = now - self.last_time
        if self.type == "f":
            md = self.mps * dt
            self.pub_ticks(self.type, round(md,2))
        elif self.type == "r":
            angle = self.aps * dt
            self.pub_ticks(self.type, round(angle,1))
            



con = None
if __name__ == '__main__':
    try:
        rospy.init_node("motor_controller", anonymous=True)
        con = MotorController("motor_status", "motor_command")
        #handle lethal signals in order to stop the motors if the script quits
        signal.signal(signal.SIGHUP, handler)
        signal.signal(signal.SIGQUIT, handler)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
