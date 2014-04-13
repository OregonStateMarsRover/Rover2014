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
    HOST = '192.168.0.50'# The remote host
    PORT = 35800 # The same port as used by the server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print sock.getsockname()

    print "Initalizing the controler:", joy.get_name(), "which has", joy.get_numaxes(),"axis"
    while not rospy.is_shutdown():
        pygame.event.pump()
        vert = int(joy.get_axis(1)*127)
        turn = int(joy.get_axis(0)*127)
        if abs(abs(vert) - abs(turn)) < 10:
            continue
        if abs(vert) > abs(turn):
            if vert < 0:
                send_string(sock, "f01")
            else:
                send_string(sock, "b01")
        elif abs(vert) < abs(turn):
            if turn > 0:
                send_string(sock, "r01")
            else:
                send_string(sock, "r359")

        clock.tick(10)


    sock.close()
