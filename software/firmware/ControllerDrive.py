import os
import serial
import time
import pygame
from serial.tools import list_ports
from pygame.locals import *

serbaud = 9600
sertimeout = 3

RoverDrivePortString = "ID: MainDrive"
RoverDrivePort = ""

def list_serialPorts():
  for port in list_ports.comports():
    yield port[0]
    
def get_controlPort(checkString):
  for port in list(list_serialPorts()):
    try:
      ser = serial.Serial(port, serbaud, timeout=sertimeout)
      multilinecheck = []
      for i in range(0,2):
	multilinecheck.append(ser.readline())
	time.sleep(2)
      if any(checkString in string for string in multilinecheck):
	  print "Port Found: " + port
	  ser.close()
	  return port
    except serial.SerialException:
      pass

def SendPacket(SerPort, left, right, estop):  
      SerPort.write(chr(255))
      SerPort.write(chr(estop))
      SerPort.write(chr(left))
      SerPort.write(chr(right))
      SerPort.write(chr(0 ^ left ^ right))
      SerPort.write(chr(255))
      
if __name__ == '__main__':
  pygame.init()
  pygame.joystick.init()
  MyJoystick = pygame.joystick.Joystick(0)
  MyJoystick.init()
  
  RoverDrivePort = get_controlPort(RoverDrivePortString)
  DrvSer = serial.Serial(RoverDrivePort, serbaud, timeout=sertimeout)
  time.sleep(2)
  DrvSer.write(chr(68))
  time.sleep(2)
  
  writenull = open(os.devnull, 'w')
 
  while(1):
    pygame.event.get()
    left = int((MyJoystick.get_axis(1)* -127) + 127)
    right = int((MyJoystick.get_axis(4)* -127) + 127)
    estop = not (int(MyJoystick.get_button(5)))
    os.system('clear')
    print "Left Value: " + str(left)
    print "Right Value: " + str(right)
    print "E-Stop: " + str(estop)
    if(DrvSer.read(1) == 'r'):
      SendPacket(DrvSer, left, right, estop)
      time.sleep(.02);
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 