#socat /dev/ttyS0,raw,echo=0,crnl /dev/ttyS1,raw,echo=0,crnl
import serial

def server():
	port = "/dev/tty0"
	s = serial.Serial(port)
	while 1:
		line = s.readline()
		while line:
			print "read:", read
			line = s.readline(v)
		s.write("r");
	s.close();

server()
