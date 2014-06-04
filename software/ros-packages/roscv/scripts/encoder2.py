#!/usr/bin/env python
import rospy
import struct
import time

import os
import serial
from serial.tools import list_ports
from std_msgs.msg import String


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

    def inWaiting(self):
        return self.ser.inWaiting()

    def readline(self):
        return self.ser.readline()

class EncoderReader():
    def __init__(self):
        self.motor_action_state = 'f' 
        self.ticks_left = 0
        self.ticks_right = 0
        self.ser = SerialHandler()
        self.ser.get_control_port("ID: DriveFeedback")
        self.ser.write("r")
        rospy.Subscriber("/motor_action", String, self.motor_action_callback)
        self.pub = rospy.Publisher("/encoder",String)
        self.prev_data = self.get_data() 
 
    def motor_action_callback(self,data):
        self.motor_action_state = data.data 
   
    def encoder_data(self):
        data = self.get_data()
        if abs(data - self.prev_data) > 1000:
            if self.prev_data > 10000:
                delta = (4294967295 - self.prev_data) + data 
            else:
                delta = -(4294967295 - data) - self.prev_data
        else:
            delta = data - self.prev_data
        print "delta:", delta
        self.prev_data = data 
        if self.motor_action_state in 'fb':
            self.ticks_left = self.ticks_left + delta
            self.ticks_right = self.ticks_right + delta
        elif self.motor_action_state == 'r':
            self.ticks_left = self.ticks_left + delta
            self.ticks_right = self.ticks_right - delta
        print "tick_left: ", self.ticks_left
        print "tick_right: ", self.ticks_right
        self.pub.publish("%d,%d" % (self.ticks_left, self.ticks_right)) 
          
    def get_data(self):
        
        self.send_packet()

        data = self.valid_packet()
       # print self.read_string(data[2:7])
       
        fixeddata = 0
        fixeddata |= ord(data[5])
        fixeddata |= (ord(data[4]) << 8)
        fixeddata |= (ord(data[3]) << 16)
        fixeddata |= (ord(data[2]) << 24)
        print "Value is : ", fixeddata
        return  fixeddata

    def valid_packet(self):
       # print "In valid_packet()"
        data = []
        while len(data) < 10:
            data.append(self.ser.read(1))

        while not(ord(data[0]) == 255 and ord(data[7]) == 255 and self.checksum(data) and ord(data[8]) == 13 and ord(data[9]) == 10):
            data = data[1:10]
            data.append(self.ser.read(1))
        print "Returning from valid_packet()"
        return data

    def checksum(self,data):
        print "in checksum"
        if (ord(data[1])^ord(data[2])^ord(data[3])^ord(data[4])^ord(data[5]) == ord(data[6])):
            return True
        else:
            return False
    def send_packet(self):
        """
        self.ser.write(chr(255))
        self.ser.write(chr(2))
        self.ser.write(chr(255))
        """
        self.ser.write("r")

    @staticmethod
    def read_string(string):
        #print len(string)
        test = struct.unpack("I", string[:4]) 
        packed = int("{:032b}".format(int(test[0]))[::-1], 2)
        #print test, packed

if __name__ == "__main__":
    rospy.init_node('encoder')
    encode = EncoderReader()
    encode.ser.readline()
    while True:
       encode.encoder_data()
       
    """motor_pub = rospy.Publisher("motor_command", String)
    pub = rospy.init_node("motor_command", "navigation")"""
