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
    PORT = 35800 # The same port as used by the server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print sock.getsockname()

    print "Initalizing the controler:", joy.get_name(), "which has", joy.get_numaxes(),"axis"

    send_string(sock, "controller")
    while not rospy.is_shutdown():
        pygame.event.pump()
        speed = 13
        left = int(joy.get_axis(1)*(-speed))+20
        right = int(joy.get_axis(4)*(-speed))+20

        print "LEFT: %d\tRIGHT: %d" % (left, right)

        left_s = ""
        right_s = ""
        if abs(20-left) <= 1:
            left = 20
        if abs(20-right) <= 1:
            right = 20
        left_s = "left"+str(left)
        right_s = "right"+str(right)
        send_string(sock, left_s+right_s)
        clock.tick(10)


    sock.close()
