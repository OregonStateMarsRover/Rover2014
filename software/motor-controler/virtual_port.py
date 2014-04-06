#socat /dev/ttyS0,raw,echo=0,crnl /dev/ttyS1,raw,echo=0,crnl
import serial

def server():
	port = "/dev/pts/8"
	s = serial.Serial(port)
	while 1:
		line = s.readline()
		while line:
			print "read:", read
			if line == "id":
				s.write("motor");
			line = s.readline(v)
		s.write("r");
	s.close();

server()
