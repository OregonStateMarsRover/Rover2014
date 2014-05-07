import os
import time
import pygame
import socket
from pygame.locals import *

UDP_LOCAL = "127.0.0.1"
UDP_REMOTE = "127.0.0.1"
UDP_SEND = 5006
UDP_RECEIVE = 5005

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
socket.bind((UDP_LOCAL, UDP_RECEIVE))

def SendPacket(left, right, estop):  
  Packet = chr(255) + chr(estop) + chr(left) + chr(right) + chr(estop ^ left ^ right) + chr(255)
  socket.sendto(Packet, (UDP_REMOTE, UDP_SEND))
  
if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()
  MyJoystick = pygame.joystick.Joystick(0)
  MyJoystick.init()
  data = None
  Packet = "Ready"
  while(1):
    socket.sendto(Packet, (UDP_REMOTE, UDP_SEND))
    data, addr = socket.recvfrom(1024)
    if data == "Ready":
      socket.sendto(Packet, (UDP_REMOTE, UDP_SEND))
      break
    
  while(1):
    data, addr = socket.recvfrom(1024)
    if data  == "Ready":
      pygame.event.get()
      left = int((MyJoystick.get_axis(1)* -127) + 127)
      right = int((MyJoystick.get_axis(3)* -127) + 127)
      estop = not (int(MyJoystick.get_button(5)))
      os.system('clear')
      print "Left Value: " + str(left)
      print "Right Value: " + str(right)
      print "E-Stop: " + str(estop)
      SendPacket(left, right, estop)    
      time.sleep(.015)
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 