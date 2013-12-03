import os
import time
import pygame
import socket
from pygame.locals import *

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 

def SendPacket(left, right, estop):  
  Packet = chr(255) + chr(estop) + chr(left) + chr(right) + chr(estop ^ left ^ right) + chr(255)
  socket.sendto(Packet, (UDP_IP, UDP_PORT))
  
if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()
  MyJoystick = pygame.joystick.Joystick(0)
  MyJoystick.init()
 
  while(1):
    pygame.event.get()
    left = int((MyJoystick.get_axis(1)* -127) + 127)
    right = int((MyJoystick.get_axis(4)* -127) + 127)
    estop = not (int(MyJoystick.get_button(5)))
    os.system('clear')
    print "Left Value: " + str(left)
    print "Right Value: " + str(right)
    print "E-Stop: " + str(estop)
    SendPacket(left, right, estop)    
    time.sleep(.015)
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 