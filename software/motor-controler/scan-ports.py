#http://stackoverflow.com/questions/11303850/what-is-the-cross-platform-method-of-enumerating-serial-ports-in-python-includi
#Sometimes Misses Virtual Serial Ports on Linux
import serial
from serial.tools import list_ports

def scan():
   # scan for available ports. return a list of tuples (num, name)
   available = []
   
   """for port in list_ports.comports():
	print port[0]
   for i in range(1, 256):
       try:
           s = serial.Serial("/dev/tty"+str(i))
	   s.write("id")
	   name = s.readline()
	   print name
           available.append( (i, s.portstr))
           s.close()
       except:
           pass"""
   available = [(10 , "/dev/pts/10")]
   return available

print "Found ports:"
for n,s in scan(): 
	print "(%d) %s" % (n,s)
	ser = serial.Serial(s)
	ser.write("id")
	line = ser.readline()
	print "device is:", line
# For latest version of pyserial:
# list_ports
