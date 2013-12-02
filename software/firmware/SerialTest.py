import os
import serial
import time
from operator import xor
from serial.tools import list_ports

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
    
def drive_test(SerPort):
  for x in range(127, 254):
      SerPort.write(chr(255))
      SerPort.write(chr(255))
      SerPort.write(chr(x))
      SerPort.write(chr(x))
      SerPort.write(chr(255 ^ x ^ x))
      SerPort.write(chr(255))
      time.sleep(.1)
  for x in reversed(range(127, 254)):
      SerPort.write(chr(255))
      SerPort.write(chr(255))
      SerPort.write(chr(x))
      SerPort.write(chr(x))
      SerPort.write(chr(255 ^ x ^ x))
      SerPort.write(chr(255))
      time.sleep(.1)
  for x in reversed(range(0, 127)):
      SerPort.write(chr(255))
      SerPort.write(chr(255))
      SerPort.write(chr(x))
      SerPort.write(chr(x))
      SerPort.write(chr(255 ^ x ^ x))
      SerPort.write(chr(255))
      time.sleep(.1)
  for x in range(0, 127):
      SerPort.write(chr(255))
      SerPort.write(chr(255))
      SerPort.write(chr(x))
      SerPort.write(chr(x))
      SerPort.write(chr(255 ^ x ^ x))
      SerPort.write(chr(255))
      time.sleep(.1)
  
    
if __name__ == '__main__':
  
  RoverDrivePort = get_controlPort(RoverDrivePortString)
  DrvSer = serial.Serial(RoverDrivePort, serbaud, timeout=sertimeout)
  time.sleep(2)
  DrvSer.write(chr(68))
  time.sleep(2)
  drive_test(DrvSer)