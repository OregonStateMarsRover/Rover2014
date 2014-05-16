#!/usr/bin/env python2
import os
import pygame
import socket
import time

import rospy

def send_string(sock, string):
    sock.sendall(string)
    data = sock.recv(1024)
    print 'Received', repr(data)


if __name__ == '__main__':
    #initlize contoller
    pygame.init()
    pygame.joystick.init()
    joy= pygame.joystick.Joystick(0)
    joy.init()

    clock = pygame.time.Clock()


    #set up socket
    HOST = '192.168.0.64'# The remote host
    PORT = 35801 # The same port as used by the server
    ticks = 0
    base = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    state = "up"
    grab = 0
    rover = 0
    sock.connect((HOST, PORT))
    print sock.getsockname()

    print "Initalizing the controler:", joy.get_name(), "which has", joy.get_numaxes(),"axis"
    while not rospy.is_shutdown():
        pygame.event.pump() 
        a = joy.get_button(0)
        b = joy.get_button(1)
        x = joy.get_button(2)
        y = joy.get_button(3)
        turn = int(joy.get_axis(0)*127)
        if abs(turn) > 20:
            if turn < 0:
                base -= 5
                base = max(base, 0)
            else:
                base += 5
                base = min(base, 50)
        #"command,rotation,lower,upper"
        upper = 0
        lower = 0
        rotation = base
        if a:
            state = "down"
        elif b:
            state = "up"

        if x:
            grab = 2 ^ grab

        if y:
            rover = 1 ^ rover

        if state == "down":
            upper = 80
            lower = 80
        else:
            upper = 300
            lower = 300

        print "%d,%d,%d,%d" % (grab, 0, lower, upper)
        send_string(sock, "%d,%d,%d,%d" % (grab, 0, lower, upper))
        if a or b or x:
            time.sleep(.5)
        clock.tick(20)
    sock.close()
