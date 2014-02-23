import serial

def server():
	port = "/dev/tty7"
	s = serial.Serial(port)
	while 1:
		line = s.readline()
		while line:
			print "read:", read
			line = s.readline(v)
		s.write("r");
	s.close();

server()
