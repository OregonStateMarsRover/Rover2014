CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

default: machine

machine: machine.cpp
	g++ -Wall -g machine.cpp -o machine $(CFLAGS) $(LIBS)
