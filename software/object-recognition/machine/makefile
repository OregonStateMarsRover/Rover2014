#make file to build the neural network
default: neural

neural: neural.o node.o
	g++ -Wall node.o neural.o `pkg-config opencv --cflags --libs` -o neural
neural.o: neural.cpp
	g++ -Wall -c neural.cpp

node.o: node.cpp
	g++ -Wall -c node.cpp



clean:
	rm node.o neural
