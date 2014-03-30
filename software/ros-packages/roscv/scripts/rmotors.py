#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
import itertools #Used to break apart string
import collections #used for the command queue


import os
import serial
from serial.tools import list_ports


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
    left = None
    right = None
    estop = 0

    def __init__(self):
        self.serial = SerialHandler()
        self.serial.get_control_port("ID: MainDrive")
        time.sleep(2)
        self.serial.write(chr(68))
        time.sleep(2)
        open(os.devnull, 'w')

    def maintain(self):
        self.send_packet()

    def change(self, l, r, e):
        self.left = l
        self.right = r
        self.estop = e

    def send_packet(self):
        self.serial.write(chr(255))
        self.serial.write(chr(self.estop))
        self.serial.write(chr(self.left))
        self.serial.write(chr(self.right))
        self.serial.write(chr(0 ^ self.left ^ self.right))
        self.serial.write(chr(255))
        while not self.serial.read(1) == 'r':
            pass
        time.sleep(.02)


class RosController(object):

    q = None
    pub = None
    m = None
    speed = 1.0

    def __init__(self, status_to, commands_from):
        self.q = collections.deque()
        self.pub = rospy.Publisher(status_to, String)

        self.m = Motor()

        rospy.Subscriber(commands_from, String, self.read_commands)

    def read_commands(self, commands):
        length = len(self.q)
        commands = str(commands).replace("data:", "").replace(" ", "")
        command_list = ["".join(x) for _, x in itertools.groupby(commands, key=str.isdigit)]
        print commands, command_list
        #flush the queues
        if command_list[0] == "flush":
            self.q.clear()

        if command_list[0] in "fbsr":
            self.q.extend(command_list)
        else:
            #invalid commands, we will need to handle later
            pass

        if length == 0:
            self.update()

    def meters_to_char(self, speed):
        #speed must be a positive number
        speed += 2.0
        #get the normalized ratio
        speed = 255*(speed/4.0)
        #ensure its an int where 0 <= speed <= 255
        return max(min(255, int(speed)), 0)


class MotorController(RosController):

    def update(self):
        if len(self.q) == 0:
            return
        action = self.q[0]
        value = self.q[1]
        if action == "f" or action == "b":
            spd = self.meters_to_char(self.speed
                            if action == "f" else -self.speed)
            self.m.change(spd, spd, 0)
            self.wait_distance(value)

        elif action == "s":
            #ensure the speed is between -2.0 and 2.0 m/s(which is the assumed max speed)
            self.speed = max(min(int(value), 20), -20)/10

        elif action == "r":
            value = int(value)
            if value < 180:
                self.m.change(self.meters_to_char(-.25), self.meters_to_char(.25), 0)
            else:
                self.m.change(self.meters_to_char(.25), self.meters_to_char(-.25), 0)
            self.wait_angle(value)
        #pop to elements off the deque
        self.q.popleft()
        self.q.popleft()
        self.update()

    def wait_distance(self, distance):
        start = time.time()
        while (time.time() - start) < 5:
            self.m.maintain()

    def wait_angle(self, angle):
        start = time.time()
        while (time.time() - start) < 5:
            self.m.maintain()





if __name__ == '__main__':
    try:
        rospy.init_node("motor_controller", anonymous=True)
        con = MotorController("motor_control", "motor_command")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
