#!/usr/bin/env python
#import rospy
import struct
import time

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




class EncoderReader():
    def __init__(self):
        self.ticks_left = 0
        self.ticks_right = 0
        self.ser = SerialHandler()
        self.ser.get_control_port("ID: DriveFeedback")
        self.ser.write("r")
    def get_data(self):
        self.send_packet()
        data = self.ser.read(12);
        print self.read_string(data[2:10])
    
    def send_packet(self):
        self.ser.write(chr(255))
        self.ser.write(chr(2))
        self.ser.write(chr(255))

    @staticmethod
    def read_string(string):
        print len(string)
        test = struct.unpack("I", string[:4]) 
        packed = int("{:032b}".format(int(test[0]))[::-1], 2)
        print test, packed

if __name__ == "__main__":
    encode = EncoderReader()
    while True:
        encode.get_data()
    """motor_pub = rospy.Publisher("motor_command", String)
    pub = rospy.init_node("motor_command", "navigation")"""
