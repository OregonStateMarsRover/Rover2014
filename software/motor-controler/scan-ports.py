#http://stackoverflow.com/questions/11303850/what-is-the-cross-platform-method-of-enumerating-serial-ports-in-python-includi
#Sometimes Misses Virtual Serial Ports on Linux
import serial
from serial.tools import list_ports

def scan():
   # scan for available ports. return a list of tuples (num, name)
   available = []
   for i in range(256):
       try:
           s = serial.Serial(i)
           available.append( (i, s.portstr))
           s.close()
       except serial.SerialException:
           pass
   return available

print "Found ports:"
for n,s in scan(): print "(%d) %s" % (n,s)
# For latest version of pyserial:
# list_ports
