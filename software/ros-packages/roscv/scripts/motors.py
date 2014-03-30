#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
import itertools #Used to break apart string
import math

import os
import serial
import serial.tools
import serial.tools.list_ports

class motor_controller(object):
        arm_queue = []
        motor_queue = []

        #controlers
        c_arm = None
        c_motor = None

        triggers = []
        
        motor_pub = None
        arm_pup = None
        
        #0 for motor 1 for arm
        state = 0
        state_cmd = []

        def __init__(self):
                #initalize the status publishers
                self.motor_pub = rospy.Publisher("motor_control", String)
                self.arm_pub = rospy.Publisher("arm_control", String)
                rospy.init_node("motor_controller")
                 
                (motor_port, arm_port) = self.get_ports()
                print motor_port
                self.c_motor = motor(motor_port)

                rospy.Subscriber("motor_command", String, self.read_commands)
                rospy.Subscriber("arm_command", String, self.read_commands)
                
                #run inital status publish
                self.publish_status()


        #publishes the arm and motor status messages
        def publish_status(self):
                #rospy.loginfo(busy)
                self.motor_pub.publish(str(len(self.motor_queue) > 0))
                self.arm_pub.publish(str(len(self.arm_queue) > 0))
                self.triggers.append(timer_trigger(1, self.publish_status))
        
        def read_commands(self, commands):
                build_up = len(self.motor_queue) + len(self.arm_queue)
                commands = str(commands).replace("data:", "").replace(" ", "")
                command_list = ["".join(x) for _, x in itertools.groupby(commands, key=str.isdigit)]
                #flush the queues
                if(command_list[0] == "flush"): 
                        self.motor_queue = []
                        self.arm_queue = []

                if command_list[0] in "fbsr":
                        self.motor_queue += command_list
                else:
                        self.arm_queue += command_list

                if build_up == 0:
                        self.send_next() 

        def send_next(self):
                #check if we need to switch states
                if self.state == 0  and len(self.motor_queue) < 1: self.state = 1
                if self.state == 1 and len(self.arm_queue) < 1: self.state = 0;
                
                self.state_cmd = self.motor_queue[:2]
                if self.state == 0 and len(self.motor_queue) > 1:
                        trig =self.c_motor.move(self.motor_queue[0], self.motor_queue[1], self.send_next)
                        self.motor_queue = self.motor_queue[2:]
                        self.triggers.append(trig)
                elif self.state == 1 and len(self.arm_queue) > 1:
                        trig = self.c_motor.move(self.arm_queue[0], self.arm_queue[1], self.send_next)
                        self.arm_queue = self.arm_queue[2:]
                        self.triggers.append(trig)
        
        
        def maintain(self):
                def nothing(): pass #create a dummy functuion since there is no call back
                if len(self.state_cmd) == 0: 
                        return
                
                if self.state == 0:
                        trig =self.c_motor.move(self.state_cmd[0], self.state_cmd[1], nothing)
                else:
                        trig =self.c_arm.move(self.state_cmd[0], self.state_cmd[1], nothing)



        def get_ports(self):
                ports = []
                for port in serial.tools.list_ports.comports():
                        if "ACM" in port[0]:
                                print port[0]
                                ser = serial.Serial(port[0], 9600, timeout=4)
                                ports.append(ser) 
                ports.append("1")
                return ports[:2]


        def run(self):
                for t in self.triggers:
                        #checks in the trigger is done
                        if t.done():
                                self.triggers.remove(t)
                        self.maintain() #maintains what ever the last command was

class motor(object):
        wrapper = chr(255)
        speed = 1.0 #this is 2.0 meters per second
        port = None

        def __init__(self, port):
                self.port = port
                time.sleep(2)
                print "sending drive control"
                self.port.write(chr(68))
                while port.read(1) != "r":
                        pass
                print "We are good to go"
                writenull = open(os.devnull, 'w')
                while 1:
                        self.send_byte(20, 20)

                        
        #should return a trigger that can be used to check when an operation is done
        def move(self, action, value, callback):
                if action ==  "f" or action == "b":
                        speed = self.speed if action == "f" else -self.speed
                        self.send_byte(speed, speed)
                        return encoder_trigger(speed*10, int(value), callback, self.send_byte)
                
                elif action == "s":
                        #ensure the speed is between -2.0 and 2.0 m/s(which is the assumed max speed)
                        self.speed = min(int(value), 20)
                        self.spped = max(self.speed, -20)
                        callback()
                        return trigger()

                
                elif action == "r":
                        value = int(value)
                        if value < 180:
                                self.send_byte(-.25, .25)
                        else:
                                self.send_byte(.25, -.25)
                        return rotation_trigger(value, callback)
                        
        
        def send_byte(self, left_drive, right_drive, estop=False): 
                #convert left and right speed to proper numbers
                cap = 0x7F
                left_drive = int(left_drive/20*cap+cap) #create a speed ratio then convert to hex 
                right_drive = int(right_drive/20*cap+cap) #create a speed ratio then convert to hex
                left_drive = right_drive = 200
                print left_drive, right_drive
                control_byte = "%c%c%c%c%c%c" % (chr(255), chr(estop), chr(left_drive), chr(right_drive), chr(0 ^ left_drive ^ right_drive), chr(255))
                print "Control Byte:", control_byte
                #self.port.write(control_byte)

                self.port.write(self.wrapper)
                self.port.write(chr(estop))
                self.port.write(chr(left_drive))
                self.port.write(chr(right_drive))
                self.port.write(chr(0 ^ left_drive ^ right_drive)) 
                self.port.write(self.wrapper)
                
                char = self.port.read(1)
                while char != "r":
                        print char
                        char = self.port.read(1)
                time.sleep(.02)

"""
" this is a simple interface that all triggers should inherit from this gives 
" a common inerface for all of the diffent types of triggers we could have
"""
class trigger(object):
        def done():
                return True

"""
" Timer based trigger
"""
class timer_trigger(trigger):
        start = 0
        length = 0
        call = None
        def __init__(self, length, callback):
                self.start = time.time()
                self.length = length
                self.call = callback 

        def done(self):
                if time.time()-self.start > self.length:
                        self.call()
                        return True
                else: 
                        return False


#a class for handling the rotation of the rover by x degrees
class rotation_trigger(timer_trigger):
        
        def __init__(self, angle, callback):
               self.start = time.time()
               """
               " the tangental velcity per second = .25
               " the angular = v/r(where we will assume r = .5) 
               " the deg/s = 2(180)*.25/pi
               "           = 90/pi
               """
               self.length = (90/math.pi)*angle
               self.call = callback

#a dummy class that is just a timer for now can change later
class encoder_trigger(timer_trigger):
       
        def __init__(self, speed, distance, callback, move):
                self.start = time.time()
                self.length = 10#distance/speed
                self.call = callback
                self.speed = speed
                self.move = move

        def done(self):
                if time.time()-self.start > self.length:
                        self.call()
                        return True
                else:
                        nspeed = self.speed/10
                        self.move(nspeed, nspeed)
                        return False



if __name__ == "__main__":
        try:
                m = motor_controller()
                while not rospy.is_shutdown():
                        m.run()
        except rospy.ROSInterruptException:
                pass



