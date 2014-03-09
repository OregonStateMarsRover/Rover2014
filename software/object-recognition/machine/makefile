#make file to build the neural network
default: neural

neural: neural.o node.o neural_network.o
	g++ -g -Wall node.o neural.o neural_network.o `pkg-config opencv --cflags --libs` -o neural
neural.o: neural.cpp
	g++ -g -Wall -c neural.cpp

node.o: node.cpp
	g++ -g -Wall -c node.cpp

neural_network.o: neural_network.cpp
	g++ -g -Wall -c neural_network.cpp


clean:
	rm node.o neural_network.o nueral.o
