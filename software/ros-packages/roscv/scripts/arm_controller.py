#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
import itertools #Used to break apart string
import collections #used for the command queue
import threading
from math import cos,acos,sin,asin,tan,atan
import signal
import sys

import os
import serial
from serial.tools import list_ports

"""
" TODO to fix this code:
" Switch the rospy.Timer to a threading.Timer like what is used here https://docs.python.org/2/library/threading.html#timer-objects
" Add a send stop byte in the update function when the q is empty
" Store threading timers in an array that you cancel when ever a flush is sent so that timers can't build up.
"""


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


class Arm(object):
    #TODO: change initial values?
    ready = True
    serial = None
    command = 0
    base1 = 0
    base2 = 0
    lowerAct1 = 0
    lowerAct2 = 0
    upperAct1 = 0
    upperAct2 = 0
    estop = 0


    def __init__(self):
        self.stopped = False
        self.serial = SerialHandler()
        self.serial.get_control_port("ID: ArmControl")
        time.sleep(2)
        self.serial.write('r')
        time.sleep(2)
        open(os.devnull, 'w')

    def maintain(self, event=None):
        self.send_packet()


    def send_packet(self):
        print "command: " + self.command
        print "base1: " + self.base1
        print "base2: " + self.base2
        print "lowerAct1: " + self.lowerAct1
        print "lowerAct2: " + self.lowerAct2
        print "upperAct1: " + self.upperAct1
        print "upperAct2: " + self.upperAct2


        self.serial.write(chr(255))
        self.serial.write(chr(self.command))
        self.serial.write(chr(self.base1))
        self.serial.write(chr(self.base2))
        self.serial.write(chr(self.lowerAct1))
        self.serial.write(chr(self.lowerAct2))
        self.serial.write(chr(self.upperAct1))
        self.serial.write(chr(self.upperAct2))
        self.serial.write(chr(self.command ^ self.base1 ^ self.base2 ^ self.lowerAct1 ^ self.lowerAct2 ^ self.upperAct1 ^ self.upperAct2))
        self.serial.write(chr(255))
        while not self.read_packet:
            pass
        time.sleep(.02)
    #TODO: fix for return packet and set flag for ready rover
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

    pub = None
    a = None

    def __init__(self, arm_status, commands_from):
        self.pub = rospy.Publisher(arm_status, String)
        self.a = Arm()

        rospy.Subscriber(commands_from, String, self.read_commands)

        rospy.Timer(rospy.Duration(.001), self.a.maintain)

    def read_commands(self, commands):
        commands = str(commands).replace("data:", "").replace(" ", "")
        command_list = commands.split(',')
        print commands, command_list

        if len(command_list) != 3:
            return
        for element in command_list:
            if not (str(element).isdigit()):
                return


       ## if not arm_ready():
       ##     self.pub.publish("Arm is busy")

        else:
            converted_vars = convertXYZ(command_list[0],command_list[1],command_list[2])

            if converted_vars[0] > 360 or converted_vars < 0:
                print "Bad theta"
                return
            else:
                if converted_vars[0] > 255:
                    a.base1 = 255
                    a.base2 = converted_vars[0] - 255
                else:
                    a.base1 = converted_vars[0]
                    a.base2 = 0

            if converted_vars[1] > 375 or converted_vars < 0:
                print "Bad s1"
                return
            else:
                if converted_vars[1] > 255:
                    a.lowerAct1 = 255
                    a.lowerAct2 = converted_vars[1] - 255
                else:
                    a.lowerAct1 = converted_vars[1]
                    a.lowerAct2 = 0
            if converted_vars[2] > 375 or converted_vars < 0:
                print "Bad s2"
                return
            else:
                if converted_vars[2] > 255:
                    a.upperAct1 = 255
                    a.upperAct2 = converted_vars[2] - 255
                else:
                    a.upperAct1 = converted_vars[2]
                    a.upperAct2 = 0
            a.send_packet()



#class ArmController(RosController):
    x1=3*self.sind(30);
    z1=3*cosd(30);
    h1=sqrt(x1**2+z1**2);
    M1=11.5;
    L1=20;

    x2=3*self.cosd(30);
    z2=3*self.sind(30);
    h2=sqrt(x2**2+z2**2);
    M2=12;
    L2=14;

    OS_1=0.5;
    OS_2=0.34808;

    def convertXYZ(self,x,y,z):
        theta_base = self.atand(y/x)

        s1 = 3
        s2 = 5
       # s1=sqrt((M1-(h1*cosd(90-atand(x1/z1))+((acos((L2**2-L1**2-(sqrt(((x/(cos((atand(y/x)))))-(OS_1+OS_2))**2+z**2))**2)/(-2*L1*(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2)))))+(atand(z/((x/(cosd((atand(y/x)))))-(OS_1+OS_2))))))))**2+(h1*sind(90-atand(x1/z1)+((acosd((L2**2-L1**2-(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2))**2)/(-2*L1*(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2)))))+(atand(z/((x/(cosd((atand(y/x)))))-(OS_1+OS_2)))))))**2)

       # s2=sqrt((M2+(h2*cosd(90-atand(x2/z2)-((acosd(((sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2))**2-L1**2-L2**2)/(-2*L1*L2)))+((acosd((L2**2-L1**2-(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2))**2)/(-2*L1*(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2)))))+(atand(z/((x/(cosd((atand(y/x)))))-(OS_1+OS_2)))))-180))))**2+(h2*sind(90-atand(x2/z2)-((acosd(((sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2))**2-L1**2-L2**2)/(-2*L1*L2)))+((acosd((L2**2-L1**2-(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2))**2)/(-2*L1*(sqrt(((x/(cosd((atand(y/x)))))-(OS_1+OS_2))**2+z**2)))))+(atand(z/((x/(cosd((atand(y/x)))))-(OS_1+OS_2)))))-180)))**2)

        return [theta_base,s1,s2]

    def atand(self, val):
        return degrees(atan(val))
    def acosd(self, val):
        return degrees(acos(val))
    def asind(self, val):
        return degrees(asin(val))
    def cosd(self, val):
        return degrees(cos(val))
    def sind(self, val):
        return degrees(sin(val))
    def tand(self, val):
        return degrees(tan(val))



'''class MotorStopperForward(threading.Thread):
    def __init__(self, update, distance):
        threading.Thread.__init__(self)
        self.update = update
        self.event = threading.Event()
    def run(self):
        while not self.event.is_set():
            self.event.wait(.25)
'''



con = None
if __name__ == '__main__':
    try:
        rospy.init_node("arm_controller", anonymous=True)
        con = ArmController("arm_controller", "arm_command")
        #handle lethal signals in order to stop the motors if the script quits
        signal.signal(signal.SIGHUP, handler)
        signal.signal(signal.SIGQUIT, handler)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


